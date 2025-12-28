#include <opencv2/opencv.hpp>
#include <iostream>
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

using namespace cv;
using namespace std;

static const int PORT = 9999;
static const int BUF_SIZE = 65536;
static const uint32_t MAGIC = 0xA0B0C0D0;

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

        // 토큰화
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

// ---- minimal overlay (top-left only) ----
static void drawLabelTopLeft(Mat& img, const string& line1, const string& line2) {
    double fontScale = 0.8;
    int thickness = 2;
    int baseline = 0;

    Size t1 = getTextSize(line1, FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
    Size t2 = getTextSize(line2, FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);

    int margin = 10;
    int pad = 6;
    int lineGap = 6;

    int w = max(t1.width, t2.width) + pad * 2;
    int h = (t1.height + t2.height) + pad * 2 + lineGap;

    Rect box(margin, margin, min(w, img.cols - margin - 1), min(h, img.rows - margin - 1));
    rectangle(img, box, Scalar(0, 0, 0), FILLED);

    Point p1(margin + pad, margin + pad + t1.height);
    Point p2(margin + pad, margin + pad + t1.height + lineGap + t2.height);

    putText(img, line1, p1, FONT_HERSHEY_SIMPLEX, fontScale, Scalar(0, 255, 0), thickness);
    putText(img, line2, p2, FONT_HERSHEY_SIMPLEX, fontScale, Scalar(0, 255, 0), thickness);
}

// ----------------- chunk reassembly -----------------
struct FrameBuf {
    uint16_t chunkCount = 0;
    vector<vector<uint8_t>> chunks;
    vector<uint8_t> got;
    int gotCount = 0;
    chrono::steady_clock::time_point t0;
};

static void printUsage(const char* prog) {
    cerr << "Usage: " << prog << " [phyIface=wlan0] [originatorMac(optional)]\n"
         << "  - phyIface     : RSSI를 읽을 무선 인터페이스 (wlan0 또는 wlx...)\n"
         << "  - originatorMac: (권장) 송신 Pi의 무선 MAC(wlan0 MAC). 있으면 멀티홉에서 TQ/relay가 정확해짐\n\n"
         << "예) sudo " << prog << " wlan0 2c:cf:67:8c:2a:7b\n";
}

int main(int argc, char** argv) {
    static const char* DEFAULT_PHY_IFACE = "wlan0";
    static const char* DEFAULT_ORIGINATOR_MAC = "2c:cf:67:8c:2a:7b"; // 송신 Pi wlan0 MAC

    string phyIface = DEFAULT_PHY_IFACE;
    string originatorMac = DEFAULT_ORIGINATOR_MAC;

    if (argc >= 2) phyIface = argv[1];
    if (argc >= 3) originatorMac = argv[2];
    if (argc >= 2 && (string(argv[1]) == "-h" || string(argv[1]) == "--help")) {
        printUsage(argv[0]);
        return 0;
    }

    string hopMac = "";
    int hopRssiDbm = 0;
    BatRoute route;

    string line1 = "HOP RSSI: N/A";
    string line2 = "TQ: N/A";

    auto lastStatusUpdate = chrono::steady_clock::now();

    // 렉 누적 방지(너무 길면 프레임 버림)
    const int FRAME_TIMEOUT_MS = 350;
    const size_t MAX_INFLIGHT_FRAMES = 32;
    uint32_t latestShown = 0;

    // UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return -1; }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    int rcvbuf = 2 * 1024 * 1024;  // 8MB -> 2MB (지연/메모리 균형)
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in myAddr{}, senderAddr{};
    myAddr.sin_family = AF_INET;
    myAddr.sin_port = htons(PORT);
    myAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (sockaddr*)&myAddr, sizeof(myAddr)) < 0) {
        perror("bind");
        return -1;
    }

    cout << "Receiver listening on UDP :" << PORT << "\n"
         << "phyIface=" << phyIface;
    if (!originatorMac.empty()) cout << ", originatorMac=" << originatorMac;
    cout << endl;

    unordered_map<uint32_t, FrameBuf> frames;
    vector<uint8_t> buf(BUF_SIZE);
    socklen_t addrLen = sizeof(senderAddr);

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
        // 1) originator가 있으면 batctl o로 nexthop/TQ 확보
        hopMac.clear();
        route = BatRoute{};
        bool haveRoute = false;

        if (!originatorMac.empty() && isMac(originatorMac)) {
            route = getBatRouteToOriginator(originatorMac);
            if (route.ok) {
                haveRoute = true;
                hopMac = route.nexthop;

                // relay 정보는 2줄 오버레이에만 최소로 반영
                string relay = (route.nexthop == route.originator) ? "direct" : ("via " + route.nexthop.substr(0, 8) + "..");
                if (route.tq >= 0) line2 = "TQ: " + to_string(route.tq) + "/255 (" + relay + ")";
                else line2 = "TQ: N/A (" + relay + ")";
            } else {
                line2 = "TQ: N/A (route?)";
            }
        }

        // 2) route가 없으면 batctl n / iw dump로 1-hop 이웃 하나 선택
        if (hopMac.empty()) {
            hopMac = getBestNeighborMac();
            if (hopMac.empty()) hopMac = getFirstStationMac(phyIface);
            if (!hopMac.empty()) {
                // relay는 모르니 hop만 표기
                line2 = "TQ: N/A (hop " + hopMac.substr(0, 8) + "..)";
            } else {
                line2 = "TQ: N/A";
            }
        }

        // 3) RSSI는 hopMac 기준
        bool ok = (!hopMac.empty()) && getRssiDbmFromStation(phyIface, hopMac, hopRssiDbm);
        if (ok) line1 = "HOP RSSI: " + to_string(hopRssiDbm) + " dBm";
        else    line1 = "HOP RSSI: N/A";
    };

    uint32_t recvCounter = 0;

    while (true) {
        ssize_t n = recvfrom(sock, buf.data(), buf.size(), 0, (sockaddr*)&senderAddr, &addrLen);
        if (n < 0) { perror("recvfrom"); continue; }

        // 0.5초마다 링크 상태 갱신
        auto now = chrono::steady_clock::now();
        auto ms = chrono::duration_cast<chrono::milliseconds>(now - lastStatusUpdate).count();
        if (ms > 500) {
            lastStatusUpdate = now;
            updateLinkStatus();
        }

        // 통짜 JPEG
        if (n >= 2 && buf[0] == 0xFF && buf[1] == 0xD8) {
            vector<uchar> data(buf.begin(), buf.begin() + n);
            Mat frame = imdecode(data, IMREAD_COLOR);
            if (!frame.empty()) {
                drawLabelTopLeft(frame, line1, line2);
                imshow("Receiver", frame);
                if (waitKey(1) == 'q') break;
            }
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

            Mat frame = imdecode(jpg, IMREAD_COLOR);
            if (!frame.empty()) {
                latestShown = max(latestShown, frameId);
                drawLabelTopLeft(frame, line1, line2);
                imshow("Receiver", frame);
            }
            frames.erase(frameId);
            if (waitKey(1) == 'q') break;
        }
    }

    close(sock);
    return 0;
}
