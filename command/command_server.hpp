#pragma once

#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>

#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

class CommandServerWin
{
public:
    explicit CommandServerWin(uint16_t port = 7777)
        : _port(port)
    {
    }

    bool start()
    {
        if (_running.load()) return true;

        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        {
            std::cerr << "[CommandServerWin] WSAStartup failed.\n";
            return false;
        }

        _running.store(true);
        _thread = std::thread([this] { thread_main(); });
        return true;
    }

    void stop()
    {
        _running.store(false);

        // Closing these unblocks accept/recv quickly.
        if (_client != INVALID_SOCKET)
        {
            shutdown(_client, SD_BOTH);
            closesocket(_client);
            _client = INVALID_SOCKET;
        }

        if (_listen != INVALID_SOCKET)
        {
            closesocket(_listen);
            _listen = INVALID_SOCKET;
        }

        if (_thread.joinable())
            _thread.join();

        WSACleanup();
    }

    // Non-blocking; safe to call from sim thread.
    // Message should be a single line (we will append '\n' if missing).
    void send_reply(std::string msg)
    {
        if (msg.empty()) return;
        if (msg.back() != '\n') msg.push_back('\n');

        std::lock_guard<std::mutex> lock(_txMx);

        // Optional: bound the queue so it can't grow forever
        if (_txQueue.size() > 512)
            _txQueue.pop_front();

        _txQueue.push_back(std::move(msg));
    }

    ~CommandServerWin() { stop(); }

    std::optional<std::string> try_pop_command()
    {
        std::lock_guard<std::mutex> lock(_mx);
        if (_queue.empty()) return std::nullopt;
        std::string s = std::move(_queue.front());
        _queue.pop();
        return s;
    }

private:
    void thread_main()
    {
        // Create/bind/listen once. If it fails, retry until stop().
        while (_running.load())
        {
            if (!ensure_listening_socket())
            {
                Sleep(500);
                continue;
            }

            // accept() blocks only this background thread.
            sockaddr_in clientAddr{};
            int clen = sizeof(clientAddr);
            SOCKET s = ::accept(_listen, (sockaddr*)&clientAddr, &clen);

            if (!_running.load())
                break;

            if (s == INVALID_SOCKET)
            {
                // If listen socket got closed/recreated, accept can fail.
                Sleep(100);
                continue;
            }

            _client = s; // best-effort visible for stop()

            // Read commands until client disconnects.
            handle_client(_client);

            // Cleanup client and continue accepting the next one.
            shutdown(_client, SD_BOTH);
            closesocket(_client);
            _client = INVALID_SOCKET;
        }
    }

    bool ensure_listening_socket()
    {
        if (_listen != INVALID_SOCKET) return true;

        _listen = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (_listen == INVALID_SOCKET)
        {
            std::cerr << "[CommandServerWin] socket() failed.\n";
            return false;
        }

        BOOL reuse = TRUE;
        setsockopt(_listen, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(_port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only

        if (::bind(_listen, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            std::cerr << "[CommandServerWin] bind() failed. Port in use?\n";
            closesocket(_listen);
            _listen = INVALID_SOCKET;
            return false;
        }

        if (::listen(_listen, 4) == SOCKET_ERROR)
        {
            std::cerr << "[CommandServerWin] listen() failed.\n";
            closesocket(_listen);
            _listen = INVALID_SOCKET;
            return false;
        }

        std::cerr << "[CommandServerWin] Listening on 127.0.0.1:" << _port << "\n";
        return true;
    }

    void push_command(std::string cmd)
    {
        while (!cmd.empty() && (cmd.back() == '\n' || cmd.back() == '\r'))
            cmd.pop_back();
        if (cmd.empty()) return;

        std::lock_guard<std::mutex> lock(_mx);
        _queue.push(std::move(cmd));
    }

    bool flush_replies(SOCKET s)
    {
        std::deque<std::string> batch;
        {
            std::lock_guard<std::mutex> lock(_txMx);
            if (_txQueue.empty()) return true;
            batch.swap(_txQueue);
        }

        for (auto& m : batch)
        {
            const char* data = m.data();
            int remaining = (int)m.size();

            while (remaining > 0 && _running.load())
            {
                int sent = ::send(s, data, remaining, 0);
                if (sent <= 0)
                {
                    // Best-effort: put unsent messages back to front
                    std::lock_guard<std::mutex> lock(_txMx);
                    // Put current and remaining back in order
                    _txQueue.push_front(std::string(data, data + remaining));
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
        // 10–20ms is usually fine for interactive tooling.
        const long kSelectTimeoutUsec = 20 * 1000; // 20ms

        for (;;)
        {
            if (!_running.load()) return;

            // 1) Flush any pending replies first (so responses are not delayed)
            if (!flush_replies(s))
            {
                std::cerr << "[CommandServerWin] send failed (client disconnected?)\n";
                return;
            }

            // 2) Wait for readability (incoming data) with timeout
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(s, &rfds);

            timeval tv;
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

            // 3) Read available incoming data (recv will not block if select said readable)
            int n = ::recv(s, tmp.data(), (int)tmp.size(), 0);
            if (n <= 0)
            {
                std::cerr << "[CommandServerWin] Client disconnected.\n";
                return;
            }

            buffer.append(tmp.data(), tmp.data() + n);

            // 4) Extract newline-delimited commands
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



private:
    uint16_t _port = 7777;

    std::atomic<bool> _running{ false };
    std::thread _thread;

    SOCKET _listen = INVALID_SOCKET;
    std::atomic<SOCKET> _client{ INVALID_SOCKET };

    std::mutex _mx;
    std::queue<std::string> _queue;

    std::mutex _txMx;
    std::deque<std::string> _txQueue;
};
