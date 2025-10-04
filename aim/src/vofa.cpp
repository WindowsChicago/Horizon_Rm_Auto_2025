#include "../include/vofa.h"

SendToVofa toVofa = {
    .TAIL = {0x00, 0x00, 0x80, 0x7f}};
ReceiveFromVofa rfvofa;

[[noreturn]] void FuncSendToVofa()
{
    int sockfd;
    struct sockaddr_in servaddr;
    socklen_t ServeraddrLen = sizeof(servaddr);
    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        std::cerr << "Socket creation failed..." << std::endl;
        close(sockfd);
    }
    // 设置服务器地址和端口（Windows机器的IP地址和端口）
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); // Windows机器的IP地址
    servaddr.sin_port = htons(7890);             // Windows机器的端口号
    if (bind(sockfd, (sockaddr *)&servaddr, (socklen_t)sizeof(servaddr)) < 0)
    {
        perror("bind error!\n");
        close(sockfd);
    }

    if (recvfrom(sockfd, &rfvofa, sizeof(rfvofa), 0, (struct sockaddr *)&servaddr, &ServeraddrLen) <= 0)
    {
        // 关闭套接字
        close(sockfd);
    }
    // 在循环中持续发送共用体数据
    while (true)
    {
        if ((sendto(sockfd, &toVofa, sizeof(toVofa), 0, (struct sockaddr *)&servaddr, sizeof(servaddr))) <= 0)
        {
            // 关闭套接字
            close(sockfd);
        }
        usleep(100);
    }
}