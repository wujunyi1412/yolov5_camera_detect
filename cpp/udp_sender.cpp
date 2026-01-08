// ------------------------------
// udp_sender.cpp
#include "udp_sender.hpp"
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

UdpSender::UdpSender(const std::string& target_ip, int target_port)
    : sockfd_(-1) {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        perror("socket");
        return;
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(target_port);

    if (inet_aton(target_ip.c_str(), &target_addr_.sin_addr) == 0) {
        std::cerr << "Invalid IP address: " << target_ip << std::endl;
        close(sockfd_);
        sockfd_ = -1;
    }
}

UdpSender::~UdpSender() {
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

bool UdpSender::send(const std::string& msg) {
    if (sockfd_ < 0) return false;

    ssize_t ret = sendto(sockfd_,
                         msg.c_str(),
                         msg.size(),
                         0,
                         reinterpret_cast<struct sockaddr*>(&target_addr_),
                         sizeof(target_addr_));
    return ret == static_cast<ssize_t>(msg.size());
}

/*
Usage example (in your detection code):

#include "udp_sender.hpp"

UdpSender sender("192.168.1.50", 5005);

// when target detected
sender.send("PRESS");
*/
