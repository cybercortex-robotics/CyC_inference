#include "CSignalingServer.h"
#include <spdlog/spdlog.h>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc == 2)
    {
        if (argv[1] == "-h")
        {
            std::cout << "Usage: ./signaling_server [port_plain port_tls]" << std::endl;
            std::cout << std::endl;
            return EXIT_SUCCESS;
        }
    }

    std::string port_plain = "1919";
    std::string port_tls = "1920";

    if (argc == 3)
    {
        port_plain = argv[1];
        port_tls = argv[2];
    }

    CSignalingServer signalingServer;
    signalingServer.run(port_plain, port_tls);

    while(true)
    {
        std::this_thread::sleep_for((std::chrono::seconds(1)));
        if (!signalingServer.is_running())
        {
            spdlog::warn("Restarting signaling server...");
            signalingServer.run(port_plain, port_tls);
        }
    }

    return EXIT_SUCCESS;
}
