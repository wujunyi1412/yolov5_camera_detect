#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

const int localPort = 5005;
const std::string triggerCmd = "PRESS";
const int pressTime = 300;   // ms
const int cooldown = 3000;   // ms

int main() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) { perror("socket"); return 1; }

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(localPort);

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }

    std::cout << "UDP simulator listening on port " << localPort << std::endl;

    auto lastTrigger = std::chrono::steady_clock::now() - std::chrono::milliseconds(cooldown);

    while (true) {
        char buf[64];
        struct sockaddr_in srcAddr{};
        socklen_t socklen = sizeof(srcAddr);
        int len = recvfrom(sockfd, buf, sizeof(buf) - 1, 0,
                           (struct sockaddr*)&srcAddr, &socklen);
        if (len > 0) {
            buf[len] = '\0';
            std::string msg(buf);

            auto now = std::chrono::steady_clock::now();
            if (msg == triggerCmd &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTrigger).count() > cooldown) {
                lastTrigger = now;
                std::cout << "[SIM] Trigger received! Press servo." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(pressTime));
                std::cout << "[SIM] Servo returned to idle." << std::endl;
            }
        }
    }

    close(sockfd);
    return 0;
}
