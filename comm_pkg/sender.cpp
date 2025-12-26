// sender.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <cctype>
#include <cstring>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace cv;
using namespace std;

// ================== 설정 ==================
static const char* DEFAULT_PEER_IP = "10.50.0.10"; ////"192.168.1.10";
static const int   DEFAULT_PORT    = 9999;

static const char* IFACE = "wlan0";     // RSSI 측정용 인터페이스
static const int   WIDTH  = 1280;       // 720p
static const int   HEIGHT = 720;
static const int   JPEG_QUALITY = 60;   // 50~70 권장(속도/화질 타협)

static const int   UDP_PAYLOAD = 1200;  // MTU(1500) 고려, 헤더 포함하면 1200 정도가 안정적
static const uint32_t MAGIC = 0xA0B0C0D0;

// ============== UDP 분할 전송 헤더 ==============
#pragma pack(push, 1)
struct UdpChunkHeader {
    uint32_t magic;      // MAGIC
    uint32_t frame_id;   // 프레임 번호
    uint16_t chunk_id;   // 0..chunk_count-1
    uint16_t chunk_count;// 전체 조각 수
    uint16_t payload_len;// 이 조각 payload 길이
};
#pragma pack(pop)

// ============== 유틸: 명령 실행 ==============
static string runCmd(const string& cmd) {
    string data;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "";
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe)) data += buf;
    pclose(pipe);
    return data;
}

// peer IP로 ARP 테이블에서 peer MAC(lladdr) 얻기
static string getPeerMacByIp(const string& iface, const string& peerIp) {
    string out = runCmd("ip neigh show " + peerIp + " dev " + iface + " 2>/dev/null");
    // 예: "192.168.1.10 lladdr 2c:cf:67:8c:2a:13 REACHABLE"
    auto pos = out.find("lladdr ");
    if (pos == string::npos) return "";
    pos += 7;
    if (pos + 17 > out.size()) return "";
    return out.substr(pos, 17);
}

// iw station get/dump에서 signal(dBm) 파싱
static bool getRssiDbmIBSS(const string& iface, const string& peerMac, int& rssiDbm) {
    string out = runCmd("iw dev " + iface + " station get " + peerMac + " 2>/dev/null");
    if (out.empty()) {
        out = runCmd("iw dev " + iface + " station dump 2>/dev/null");
        if (out.empty()) return false;
    }

    istringstream iss(out);
    string line;
    while (getline(iss, line)) {
        if (line.find("signal:") != string::npos) {
            for (size_t i = 0; i < line.size(); i++) {
                if (line[i] == '-' && i + 1 < line.size() && isdigit((unsigned char)line[i + 1])) {
                    try {
                        rssiDbm = stoi(line.substr(i));
                        return true;
                    } catch (...) {
                        return false;
                    }
                }
            }
        }
    }
    return false;
}

static void drawTextTopRight(Mat& img, const string& text) {
    int baseline = 0;
    double fontScale = 0.9;
    int thickness = 2;
    Size ts = getTextSize(text, FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
    int margin = 10;
    Point org(img.cols - ts.width - margin, ts.height + margin);
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, fontScale, Scalar(0, 255, 0), thickness);
}

// ================== main ==================
int main(int argc, char** argv) {
    string peerIp = (argc >= 2) ? argv[1] : DEFAULT_PEER_IP;
    int port      = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORT;
    bool overlay = false;
    for (int i=1;i<argc;i++) if (string(argv[i])=="--overlay") overlay=true;

    // UDP 소켓
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return -1; }

    int sndbuf = 4 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    sockaddr_in peerAddr{};
    peerAddr.sin_family = AF_INET;
    peerAddr.sin_port = htons(port);
    if (inet_pton(AF_INET, peerIp.c_str(), &peerAddr.sin_addr) != 1) {
        cerr << "Invalid peer IP\n";
        return -1;
    }

    // 카메라
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) { cerr << "Camera open failed\n"; return -1; }

    cap.set(CAP_PROP_FRAME_WIDTH,  WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, HEIGHT);

    vector<int> params = { IMWRITE_JPEG_QUALITY, JPEG_QUALITY };

    // RSSI 관련
    string peerMac;
    string rssiText = "RSSI: N/A";
    auto lastRssi = chrono::steady_clock::now();

    uint32_t frameId = 0;
    cout << "Start Streaming to " << peerIp << ":" << port
         << " (" << WIDTH << "x" << HEIGHT << ", Q=" << JPEG_QUALITY << ")\n";

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // 0.5초마다 RSSI 갱신
        auto now = chrono::steady_clock::now();
        auto ms = chrono::duration_cast<chrono::milliseconds>(now - lastRssi).count();
        if (ms > 500) {
            lastRssi = now;

            if (peerMac.empty()) {
                peerMac = getPeerMacByIp(IFACE, peerIp);
                if (peerMac.empty()) {
                    // ARP 채우기 시도
                    runCmd("ping -c 1 -W 1 " + peerIp + " >/dev/null 2>&1");
                    peerMac = getPeerMacByIp(IFACE, peerIp);
                }
            }

            int dbm = 0;
            if (!peerMac.empty() && getRssiDbmIBSS(IFACE, peerMac, dbm)) {
                rssiText = "RSSI: " + to_string(dbm) + " dBm";
            } else {
                rssiText = "RSSI: N/A";
            }
        }

        // 프레임에 RSSI 우상단 오버레이(원하면 --overlay 옵션)
        if (overlay) drawTextTopRight(frame, rssiText);

        // JPEG 인코딩
        vector<uchar> jpg;
        imencode(".jpg", frame, jpg, params);

        // UDP 분할 전송
        const int totalLen = (int)jpg.size();
        const int chunkPayload = UDP_PAYLOAD;
        const uint16_t chunkCount = (uint16_t)((totalLen + chunkPayload - 1) / chunkPayload);

        for (uint16_t cid = 0; cid < chunkCount; cid++) {
            int offset = cid * chunkPayload;
            int len = min(chunkPayload, totalLen - offset);

            UdpChunkHeader h{};
            h.magic       = htonl(MAGIC);
            h.frame_id    = htonl(frameId);
            h.chunk_id    = htons(cid);
            h.chunk_count = htons(chunkCount);
            h.payload_len = htons((uint16_t)len);

            vector<uint8_t> packet(sizeof(UdpChunkHeader) + len);
            memcpy(packet.data(), &h, sizeof(UdpChunkHeader));
            memcpy(packet.data() + sizeof(UdpChunkHeader), jpg.data() + offset, len);

            ssize_t sent = sendto(sock, packet.data(), packet.size(), 0,
                                  (sockaddr*)&peerAddr, sizeof(peerAddr));
            if (sent < 0) {
                perror("sendto");
                break;
            }
        }

        frameId++;

    }

    close(sock);
    return 0;
}
