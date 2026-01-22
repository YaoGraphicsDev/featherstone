#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>

#include "command_server.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

struct CommandServerWin::Impl
{
    explicit Impl(uint16_t p) : port(p) {}

    uint16_t port = 7777;

    std::atomic<bool> running{ false };
    std::thread thread;

    SOCKET listenSock = INVALID_SOCKET;
    std::atomic<SOCKET> clientSock{ INVALID_SOCKET };

    std::mutex rxMx;
    std::queue<std::string> rxQueue;

    std::mutex txMx;
    std::deque<std::string> txQueue;

    static void trim_crlf(std::string& s)
    {
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    }

    void push_command(std::string cmd)
    {
        trim_crlf(cmd);
        if (cmd.empty()) return;

        std::lock_guard<std::mutex> lock(rxMx);
        rxQueue.push(std::move(cmd));
    }

    bool ensure_listening_socket()
    {
        if (listenSock != INVALID_SOCKET) return true;

        listenSock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listenSock == INVALID_SOCKET)
        {
            std::cerr << "[CommandServerWin] socket() failed.\n";
            return false;
        }

        BOOL reuse = TRUE;
        setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only

        if (::bind(listenSock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            std::cerr << "[CommandServerWin] bind() failed. Port in use?\n";
            closesocket(listenSock);
            listenSock = INVALID_SOCKET;
            return false;
        }

        if (::listen(listenSock, 4) == SOCKET_ERROR)
        {
            std::cerr << "[CommandServerWin] listen() failed.\n";
            closesocket(listenSock);
            listenSock = INVALID_SOCKET;
            return false;
        }

        std::cerr << "[CommandServerWin] Listening on 127.0.0.1:" << port << "\n";
        return true;
    }

    bool flush_replies(SOCKET s)
    {
        std::deque<std::string> batch;
        {
            std::lock_guard<std::mutex> lock(txMx);
            if (txQueue.empty()) return true;
            batch.swap(txQueue);
        }

        for (auto& m : batch)
        {
            const char* data = m.data();
            int remaining = (int)m.size();

            while (remaining > 0 && running.load())
            {
                int sent = ::send(s, data, remaining, 0);
                if (sent <= 0)
                {
                    // Socket broke; drop (or you could requeue if you prefer)
                    return false;
                }
                data += sent;
                remaining -= sent;
            }
        }
        return true;
    }

    void handle_client(SOCKET s)
    {
        std::string buffer;
        buffer.reserve(4096);
        std::vector<char> tmp(1024);

        // Wake up periodically even if no incoming data, to flush replies.
        const long kSelectTimeoutUsec = 20 * 1000; // 20ms

        for (;;)
        {
            if (!running.load()) return;

            // Flush replies first so responses are not delayed
            if (!flush_replies(s))
            {
                std::cerr << "[CommandServerWin] send failed (client disconnected?)\n";
                return;
            }

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(s, &rfds);

            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = kSelectTimeoutUsec;

            int r = ::select(0, &rfds, nullptr, nullptr, &tv);
            if (r == SOCKET_ERROR)
            {
                std::cerr << "[CommandServerWin] select() error\n";
                return;
            }

            if (r == 0)
            {
                // timeout -> loop again to flush replies
                continue;
            }

            int n = ::recv(s, tmp.data(), (int)tmp.size(), 0);
            if (n <= 0)
            {
                std::cerr << "[CommandServerWin] Client disconnected.\n";
                return;
            }

            buffer.append(tmp.data(), tmp.data() + n);

            for (;;)
            {
                size_t nl = buffer.find('\n');
                if (nl == std::string::npos) break;

                std::string line = buffer.substr(0, nl + 1);
                buffer.erase(0, nl + 1);
                push_command(std::move(line));
            }

            if (buffer.size() > (1u << 20))
                buffer.clear();
        }
    }

    void thread_main()
    {
        while (running.load())
        {
            if (!ensure_listening_socket())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            sockaddr_in clientAddr{};
            int clen = sizeof(clientAddr);
            SOCKET s = ::accept(listenSock, (sockaddr*)&clientAddr, &clen);

            if (!running.load())
                break;

            if (s == INVALID_SOCKET)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            clientSock.store(s);
            handle_client(s);

            shutdown(s, SD_BOTH);
            closesocket(s);
            clientSock.store(INVALID_SOCKET);
        }
    }
};

CommandServerWin::CommandServerWin(uint16_t port)
    : _impl(std::make_unique<Impl>(port))
{
}

CommandServerWin::~CommandServerWin()
{
    stop();
}

bool CommandServerWin::start()
{
    if (_impl->running.load()) return true;

    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::cerr << "[CommandServerWin] WSAStartup failed.\n";
        return false;
    }

    _impl->running.store(true);
    _impl->thread = std::thread([p = _impl.get()] { p->thread_main(); });
    return true;
}

void CommandServerWin::stop()
{
    if (!_impl->running.exchange(false))
        return;

    // Unblock recv() quickly if connected
    SOCKET cs = _impl->clientSock.load();
    if (cs != INVALID_SOCKET)
    {
        shutdown(cs, SD_BOTH);
        closesocket(cs);
        _impl->clientSock.store(INVALID_SOCKET);
    }

    // Unblock accept() quickly
    if (_impl->listenSock != INVALID_SOCKET)
    {
        closesocket(_impl->listenSock);
        _impl->listenSock = INVALID_SOCKET;
    }

    if (_impl->thread.joinable())
        _impl->thread.join();

    WSACleanup();
}

std::optional<std::string> CommandServerWin::try_pop_command()
{
    std::lock_guard<std::mutex> lock(_impl->rxMx);
    if (_impl->rxQueue.empty()) return std::nullopt;
    std::string s = std::move(_impl->rxQueue.front());
    _impl->rxQueue.pop();
    return s;
}

void CommandServerWin::send_reply(std::string msg)
{
    if (msg.empty()) return;
    if (msg.back() != '\n') msg.push_back('\n');

    std::lock_guard<std::mutex> lock(_impl->txMx);

    if (_impl->txQueue.size() > 512)
        _impl->txQueue.pop_front();

    _impl->txQueue.push_back(std::move(msg));
}
