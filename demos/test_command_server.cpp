#include "command_server.h"
#include <iostream>
#include <thread>

int main() {
    CommandServerWin server(7777);
    server.start();

    int frame_id = 0;
    // simulation loop
    while (true)
    {
        while (auto cmd = server.try_pop_command())
        {
            std::cout << "[command] " << *cmd << std::endl;
            server.send_reply("Relply to command: " + *cmd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        std::cout << "Frame " << frame_id++ << std::endl;
    }

    // on shutdown:
    server.stop();
    return 0;
}