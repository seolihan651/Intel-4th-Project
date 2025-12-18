#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

#define PORT 9999
#define BUFFER_SIZE 65536 // 64KB

using namespace cv;
using namespace std;

int main() {
    // 1. UDP 소켓 생성
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        cerr << "Socket creation failed" << endl;
        return -1;
    }

    // 2. 바인딩 (수신 대기)
    struct sockaddr_in myAddr, senderAddr;
    memset(&myAddr, 0, sizeof(myAddr));
    myAddr.sin_family = AF_INET;
    myAddr.sin_port = htons(PORT);
    myAddr.sin_addr.s_addr = htonl(INADDR_ANY); // 모든 IP에서 수신

    if (bind(sock, (struct sockaddr*)&myAddr, sizeof(myAddr)) < 0) {
        cerr << "Bind failed" << endl;
        return -1;
    }

    // UDP 수신 버퍼 크기 늘리기 (OS 설정에 따라 다름, 필수 아님)
    int rcvbuf = 1024 * 1024; // 1MB
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    cout << "Waiting for video stream on port " << PORT << "..." << endl;

    char buffer[BUFFER_SIZE];
    socklen_t addrLen = sizeof(senderAddr);

    while (true) {
        // 3. 데이터 수신
        ssize_t recvLen = recvfrom(sock, buffer, BUFFER_SIZE, 0, 
                                   (struct sockaddr*)&senderAddr, &addrLen);
        
        if (recvLen > 0) {
            // 4. JPEG 디코딩
            // 받은 char 배열을 OpenCV 매트릭스로 변환 (복사 없이 포인터 활용 가능하지만 안전하게 벡터로 변환)
            vector<uchar> data(buffer, buffer + recvLen);
            Mat frame = imdecode(data, IMREAD_COLOR);

            if (!frame.empty()) {
                imshow("Ad-hoc Receiver (C++)", frame);
            }
        }

        if (waitKey(1) == 'q') break;
    }

    close(sock);
    return 0;
}