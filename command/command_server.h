#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <memory>

class CommandServerWin
{
public:
    explicit CommandServerWin(uint16_t port = 7777);
    ~CommandServerWin();

    bool start();
    void stop();

    // Non-blocking; safe to call from sim thread
    std::optional<std::string> try_pop_command();

    // Optional reply; safe to call from sim thread
    void send_reply(std::string msg);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
