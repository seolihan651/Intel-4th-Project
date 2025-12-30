#include "mesh_rx_thread.h"
#include <QDebug>

#include <vector>
#include <unordered_map>
#include <chrono>
#include <cstring>
#include <sstream>
#include <cctype>
#include <algorithm>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>

#include <algorithm>
#include <unordered_map>
#include <sstream>

using namespace std;

namespace {
constexpr int BUF_SIZE = 65536;
constexpr uint32_t MAGIC = 0xA0B0C0D0;

// receiver_fixed_tuned 튜닝 반영
constexpr int RCVBUF_BYTES = 2 * 1024 * 1024;    // 2MB
constexpr int LINK_UPDATE_MS = 500;             // 0.5초
constexpr int EMIT_MIN_MS = 40;                 // 25fps 제한
constexpr int FRAME_TIMEOUT_MS = 350;
constexpr size_t MAX_INFLIGHT_FRAMES = 32;

#pragma pack(push, 1)
struct UdpChunkHeader {
    uint32_t magic;
    uint32_t frame_id;
    uint16_t chunk_id;
    uint16_t chunk_count;
    uint16_t payload_len;
};
#pragma pack(pop)

static inline uint32_t ntoh32(uint32_t x) { return ntohl(x); }
static inline uint16_t ntoh16(uint16_t x) { return ntohs(x); }

// ----------------- cmd utils -----------------
static string runCmd(const string& cmd) {
    string data;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "";
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe)) data += buf;
    pclose(pipe);
    return data;
}

static bool isMac(const string& s) {
    if (s.size() != 17) return false;
    for (int i = 0; i < 17; i++) {
        if (i % 3 == 2) { if (s[i] != ':') return false; }
        else { if (!isxdigit((unsigned char)s[i])) return false; }
    }
    return true;
}

static string trim(const string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

static bool parseSignalLine(const string& text, int& dbm) {
    istringstream iss(text);
    string line;
    while (getline(iss, line)) {
        if (line.find("signal:") == string::npos) continue;
        for (size_t i = 0; i < line.size(); i++) {
            if (line[i] == '-' && i + 1 < line.size() && isdigit((unsigned char)line[i + 1])) {
                try { dbm = stoi(line.substr(i)); return true; }
                catch (...) { return false; }
            }
        }
    }
    return false;
}

static bool getRssiDbmFromStation(const string& iface, const string& mac, int& dbm) {
    if (mac.empty()) return false;
    string out = runCmd("iw dev " + iface + " station get " + mac + " 2>/dev/null");
    if (!out.empty() && parseSignalLine(out, dbm)) return true;
    return false;
}

// iw station dump에서 첫 번째 Station MAC 가져오기(최후의 fallback)
static string getFirstStationMac(const string& iface) {
    string dump = runCmd("iw dev " + iface + " station dump 2>/dev/null");
    if (dump.empty()) return "";
    size_t p = dump.find("Station ");
    if (p == string::npos) return "";
    p += 8;
    if (p + 17 > dump.size()) return "";
    string mac = dump.substr(p, 17);
    return isMac(mac) ? mac : "";
}

struct BatRoute {
    bool ok = false;
    string originator;
    string nexthop;
    int tq = -1; // 0..255
};

// batctl o에서 특정 originator 경로 파싱
static BatRoute getBatRouteToOriginator(const string& originatorMac) {
    BatRoute r;
    r.originator = originatorMac;
    if (!isMac(originatorMac)) return r;

    string out = runCmd("batctl o 2>/dev/null");
    if (out.empty()) return r;

    istringstream iss(out);
    string line;
    while (getline(iss, line)) {
        if (line.find(originatorMac) == string::npos) continue;

        string s = trim(line);
        if (s.empty()) continue;

        istringstream ls(s);
        vector<string> tok;
        string t;
        while (ls >> t) tok.push_back(t);
        if (tok.size() < 4) continue;

        size_t idx = 0;
        if (tok[0] == "*") idx = 1;
        if (idx + 3 >= tok.size()) continue;

        string org = tok[idx];
        string tqTok = tok[idx + 2];
        string next = tok[idx + 3];

        if (!isMac(org) || !isMac(next)) continue;
        if (org != originatorMac) continue;

        if (tqTok.size() >= 3 && tqTok.front() == '(' && tqTok.back() == ')') {
            try { r.tq = stoi(tqTok.substr(1, tqTok.size() - 2)); } catch (...) {}
        }

        r.ok = true;
        r.nexthop = next;
        return r;
    }
    return r;
}

// batctl n에서 last-seen 최소(=가장 최근) 이웃 MAC 찾기
static string getBestNeighborMac() {
    string out = runCmd("batctl n 2>/dev/null");
    if (out.empty()) return "";

    istringstream iss(out);
    string line;
    double bestSeen = 1e9;
    string bestMac;

    while (getline(iss, line)) {
        if (line.find("Neighbor") != string::npos) continue;
        istringstream ls(line);
        string ifname, mac, last;
        ls >> ifname >> mac >> last;
        if (!isMac(mac)) continue;
        if (last.empty() || last.back() != 's') continue;
        double seen = 0.0;
        try { seen = stod(last.substr(0, last.size() - 1)); } catch (...) { continue; }
        if (seen < bestSeen) {
            bestSeen = seen;
            bestMac = mac;
        }
    }
    return bestMac;
}

// last-seen 뽑는 함수
struct NeighborSeen {
    std::string mac;
    double seenSec; // last-seen seconds
};

static std::vector<NeighborSeen> getNeighborsWithSeen()
{
    std::vector<NeighborSeen> v;
    std::string out = runCmd("batctl n 2>/dev/null");
    if (out.empty()) return v;

    std::istringstream iss(out);
    std::string line;

    while (std::getline(iss, line)) {
        if (line.find("Neighbor") != std::string::npos) continue; // header skip

        std::istringstream ls(line);
        std::string ifname, mac, last;
        ls >> ifname >> mac >> last;

        if (!isMac(mac)) continue;
        if (last.empty() || last.back() != 's') continue;

        double seen = 0.0;
        try { seen = std::stod(last.substr(0, last.size() - 1)); }
        catch (...) { continue; }

        v.push_back({mac, seen});
    }
    return v;
}



// ---- minimal overlay (top-left only) ----
static void drawLabelTopLeft(cv::Mat& img, const string& l1, const string& l2) {
    double fontScale = 0.8;
    int thickness = 2;
    int baseline = 0;

    cv::Size t1 = cv::getTextSize(l1, cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
    cv::Size t2 = cv::getTextSize(l2, cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);

    int margin = 10;
    int pad = 6;
    int lineGap = 6;

    int w = max(t1.width, t2.width) + pad * 2;
    int h = (t1.height + t2.height) + pad * 2 + lineGap;

    cv::Rect box(margin, margin, min(w, img.cols - margin - 1), min(h, img.rows - margin - 1));
    cv::rectangle(img, box, cv::Scalar(0, 0, 0), cv::FILLED);

    cv::Point p1(margin + pad, margin + pad + t1.height);
    cv::Point p2(margin + pad, margin + pad + t1.height + lineGap + t2.height);

    cv::putText(img, l1, p1, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 255, 0), thickness);
    cv::putText(img, l2, p2, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 255, 0), thickness);
}

// ----------------- chunk reassembly -----------------
struct FrameBuf {
    uint16_t chunkCount = 0;
    vector<vector<uint8_t>> chunks;
    vector<uint8_t> got;
    int gotCount = 0;
    chrono::steady_clock::time_point t0;
};
} // namespace

#include <unordered_map>
#include <sstream>

#include <unordered_map>
#include <sstream>

static std::unordered_map<std::string, std::string> getMacToIpBat0()
{
    std::unordered_map<std::string, std::string> mac2ip;

    std::string out = runCmd("ip neigh show dev bat0 2>/dev/null");
    if (out.empty()) return mac2ip;

    std::istringstream iss(out);
    std::string line;

    while (std::getline(iss, line)) {
        // 10.10.14.79 dev bat0 lladdr 00:14:1b:... REACHABLE
        std::istringstream ls(line);
        std::string ip, dev, ifname, lladdr, mac;
        ls >> ip >> dev >> ifname >> lladdr >> mac;

        if (ip.empty() || mac.empty()) continue;
        if (ifname != "bat0") continue;
        if (lladdr != "lladdr") continue;
        if (!isMac(mac)) continue;

        mac2ip[mac] = ip;
    }
    return mac2ip;
}




// ----------------- MeshRxThread -----------------
MeshRxThread::MeshRxThread(QObject *parent)
    : QThread(parent)
{
}

void MeshRxThread::setLinkPollEnabled(bool enable) {
    linkPollEnabled = enable;
}

void MeshRxThread::snapShot() {
    QMutexLocker lk(&frameMtx);
    if (lastFrameBgr.empty()) return;

    string fname = "rx_" + to_string(cnt++) + ".jpg";
    cv::imwrite(fname, lastFrameBgr);
    qDebug() << "[Tab7] snapshot saved:" << QString::fromStdString(fname);
}

void MeshRxThread::run()
{
    // UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return; }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // receiver_fixed_tuned 튜닝: RCVBUF 2MB
    int rcvbuf = RCVBUF_BYTES;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // stop 빨리 먹게 recv timeout
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000; // 200ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in myAddr{}, senderAddr{};
    myAddr.sin_family = AF_INET;
    myAddr.sin_port = htons(port);
    myAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (sockaddr*)&myAddr, sizeof(myAddr)) < 0) {
        perror("bind");
        close(sock);
        return;
    }

    unordered_map<uint32_t, FrameBuf> frames;
    vector<uint8_t> buf(BUF_SIZE);
    socklen_t addrLen = sizeof(senderAddr);

    string hopMac;
    int hopRssiDbm = 0;
    BatRoute route;

    auto cleanupOld = [&]() {
        auto now = chrono::steady_clock::now();


        for (auto it = frames.begin(); it != frames.end();) {
            auto age = chrono::duration_cast<chrono::milliseconds>(now - it->second.t0).count();
            if (age > FRAME_TIMEOUT_MS) it = frames.erase(it);
            else ++it;
        }
        if (frames.size() > MAX_INFLIGHT_FRAMES) {
            vector<pair<uint32_t, chrono::steady_clock::time_point>> v;
            v.reserve(frames.size());
            for (auto& kv : frames) v.push_back({kv.first, kv.second.t0});
            sort(v.begin(), v.end(), [](auto& a, auto& b){ return a.second < b.second; });
            size_t removeN = frames.size() - MAX_INFLIGHT_FRAMES;
            for (size_t i = 0; i < removeN; i++) frames.erase(v[i].first);
        }
    };

    auto updateLinkStatus = [&]() {
        //if (!linkPollEnabled.load()) {
            line1 = "HOP RSSI: N/A";
            line2 = "TQ: N/A";

            // TQ 값 probar에 전달하기 위해 추가
            int tqNow =(route.ok ? route.tq : -1);

            if(tqNow !=lastTq.load())
            {
                lastTq=tqNow;
                emit linkUpdated(tqNow);
            }

    auto lastLinkUpdate = chrono::steady_clock::now() - chrono::seconds(10);
    auto lastEmit = chrono::steady_clock::now() - chrono::milliseconds(100);

          //  return;
        //}

        // receiver_fixed_tuned 로직 그대로 반영
        hopMac.clear();
        route = BatRoute{};

        // 1) originatorMac 있으면 batctl o로 nexthop/TQ 확보
        if (!originatorMac.empty() && isMac(originatorMac)) {
            route = getBatRouteToOriginator(originatorMac);
            if (route.ok) {
                hopMac = route.nexthop;
                string relay = (route.nexthop == route.originator) ? "direct"
                                                                   : ("via " + route.nexthop.substr(0, 8) + "..");
                if (route.tq >= 0) line2 = "TQ: " + to_string(route.tq) + "/255 (" + relay + ")";
                else line2 = "TQ: N/A (" + relay + ")";
            } else {
                line2 = "TQ: N/A (route?)";
            }
        }

        // 2) route 없으면 batctl n / iw dump로 1-hop 이웃 하나 선택
        if (hopMac.empty()) {
            hopMac = getBestNeighborMac();
            if (hopMac.empty()) hopMac = getFirstStationMac(phyIface);
            if (!hopMac.empty()) line2 = "TQ: N/A (hop " + hopMac.substr(0, 8) + "..)";
            else line2 = "TQ: N/A";
        }

        // 3) RSSI는 hopMac 기준
        bool ok = (!hopMac.empty()) && getRssiDbmFromStation(phyIface, hopMac, hopRssiDbm);
        if (ok) line1 = "HOP RSSI: " + to_string(hopRssiDbm) + " dBm";
        else    line1 = "HOP RSSI: N/A";
    };

    auto updateDeviceInfo = [&]() {
        // 1) batctl o 에서 MAC 목록 뽑기
        vector<string> macs;
        {
            string out = runCmd("batctl o 2>/dev/null");
            istringstream iss(out);
            string line;
            while (getline(iss, line)) {
                string s = trim(line);
                if (s.empty()) continue;

                // 토큰화해서 첫 MAC 찾기 (형태: "* 2c:cf:...  2.7s (93) ...")
                istringstream ls(s);
                string tok;
                while (ls >> tok) {
                    if (isMac(tok)) {
                        // 중복 방지
                        bool dup = false;
                        for (auto &m : macs) { if (m == tok) { dup = true; break; } }
                        if (!dup) macs.push_back(tok);
                        break;
                    }
                }
            }
        }


        auto neigh = getNeighborsWithSeen();

        neigh.erase(std::remove_if(neigh.begin(), neigh.end(),
                                   [](const NeighborSeen& n){ return n.seenSec > 60.0; }),
                    neigh.end());

        //auto mac2ip = getMacToIpBat0();

        // 2) ip neigh show dev bat0 에서 MAC->IP 맵 만들기
        unordered_map<string,string> mac2ip;
        {
            string out = runCmd("ip neigh show dev bat0 2>/dev/null");
            istringstream iss(out);
            string line;
            while (getline(iss, line)) {
                // 예: 10.10.14.79 dev bat0 lladdr 00:14:1b:... REACHABLE
                istringstream ls(line);
                string ip, dev, ifname, lladdr, mac;
                ls >> ip >> dev >> ifname >> lladdr >> mac;
                if (ip.empty() || mac.empty()) continue;
                if (ifname != "bat0") continue;
                if (lladdr != "lladdr") continue;
                if (!isMac(mac)) continue;
                mac2ip[mac] = ip;
            }
        }

        // 3) 문자열 구성 (Unknown 허용)
        QString text;
        text += QString("DEVICE INFO (%1 Devices)\n\n").arg((int)macs.size());

        int idx = 1;
        for (auto &mac : macs) {
            QString qmac = QString::fromStdString(mac);
            QString ip = "Unknown";
            auto it = mac2ip.find(mac);
            if (it != mac2ip.end()) ip = QString::fromStdString(it->second);

            text += QString("Device %1  MAC %2\n").arg(idx++).arg(qmac);
            text += QString("         IP  %1\n\n").arg(ip);
        }

        emit deviceInfoUpdated(text);
    };

    auto lastLinkUpdate = chrono::steady_clock::now() - chrono::seconds(10);
    auto lastDevUpdate  = chrono::steady_clock::now() - chrono::seconds(10);
    auto lastEmit = chrono::steady_clock::now() - chrono::milliseconds(100);

    uint32_t latestShown =0;
    uint32_t recvCounter = 0;

    while (camViewFlag.load()) {
        // receiver_fixed_tuned 튜닝: 0.5초마다 링크 갱신
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now - lastLinkUpdate).count() >= LINK_UPDATE_MS) {
            updateLinkStatus();
            lastLinkUpdate = now;
        }
        if (chrono::duration_cast<chrono::milliseconds>(now - lastDevUpdate).count() >= 1000) {
            updateDeviceInfo();
            lastDevUpdate = now;
        }


        ssize_t n = recvfrom(sock, buf.data(), buf.size(), 0, (sockaddr*)&senderAddr, &addrLen);
        if (n < 0) {
            // timeout(EAGAIN/EWOULDBLOCK)은 정상 -> 계속
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            // 그 외 에러는 로그만 찍고 계속(일시적일 수 있음)
            // perror("recvfrom");
            continue;
        }

        auto emitFrame = [&](cv::Mat &bgr) {
            // snapshot용 최신 프레임 저장
            {
                QMutexLocker lk(&frameMtx);
                lastFrameBgr = bgr.clone();
            }

            drawLabelTopLeft(bgr, line1, line2);

            // emit fps 제한(25fps)
            auto tnow = chrono::steady_clock::now();
            if (chrono::duration_cast<chrono::milliseconds>(tnow - lastEmit).count() < EMIT_MIN_MS) {
                return;
            }
            lastEmit = tnow;

            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            QImage qimg(rgb.data, rgb.cols, rgb.rows, (int)rgb.step, QImage::Format_RGB888);

            // 매우 중요: Mat 메모리와 분리
            emit frameReady(qimg.copy());
        };

        // 통짜 JPEG도 대응
        if (n >= 2 && buf[0] == 0xFF && buf[1] == 0xD8) {
            vector<uchar> data(buf.begin(), buf.begin() + n);
            cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);
            if (!frame.empty()) emitFrame(frame);
            continue;
        }

        // chunk header
        if (n < (ssize_t)sizeof(UdpChunkHeader)) continue;

        UdpChunkHeader h{};
        memcpy(&h, buf.data(), sizeof(UdpChunkHeader));
        if (ntoh32(h.magic) != MAGIC) continue;

        uint32_t frameId = ntoh32(h.frame_id);
        uint16_t cid     = ntoh16(h.chunk_id);
        uint16_t ccount  = ntoh16(h.chunk_count);
        uint16_t plen    = ntoh16(h.payload_len);

        if ((ssize_t)(sizeof(UdpChunkHeader) + plen) != n) continue;
        if (ccount == 0 || cid >= ccount) continue;

        // 너무 오래된 프레임 드롭(렉 누적 방지)
        if (latestShown > 0 && frameId + 50 < latestShown) continue;

        auto& fb = frames[frameId];
        if (fb.chunkCount == 0) {
            fb.chunkCount = ccount;
            fb.chunks.resize(ccount);
            fb.got.assign(ccount, 0);
            fb.gotCount = 0;
            fb.t0 = chrono::steady_clock::now();
        }
        if (fb.chunkCount != ccount) { frames.erase(frameId); continue; }

        if (!fb.got[cid]) {
            fb.got[cid] = 1;
            fb.gotCount++;
            fb.chunks[cid].assign(buf.begin() + sizeof(UdpChunkHeader),
                                  buf.begin() + sizeof(UdpChunkHeader) + plen);
        }

        recvCounter++;
        if ((recvCounter % 40) == 0) cleanupOld();

        // frame complete
        if (fb.gotCount == fb.chunkCount) {
            vector<uint8_t> jpg;
            size_t total = 0;
            for (auto& c : fb.chunks) total += c.size();
            jpg.reserve(total);
            for (auto& c : fb.chunks) jpg.insert(jpg.end(), c.begin(), c.end());

            cv::Mat frame = cv::imdecode(jpg, cv::IMREAD_COLOR);
            if (!frame.empty()) {
                latestShown = max(latestShown, frameId);
                emitFrame(frame);
            }
            frames.erase(frameId);
        }
    }

    close(sock);
    emit resetToDefault();
}
