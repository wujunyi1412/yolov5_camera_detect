#pragma once
#include <string>
#include <netinet/in.h>

class UdpSender {
public:
UdpSender(const std::string& target_ip, int target_port);
~UdpSender();


// send trigger message, e.g. "PRESS"
bool send(const std::string& msg);


private:
int sockfd_;
struct sockaddr_in target_addr_;
};