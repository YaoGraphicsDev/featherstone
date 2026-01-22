#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <atomic>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

static std::atomic<bool> g_running{ true };

static BOOL WINAPI ConsoleCtrlHandler(DWORD ctrlType)
{
    switch (ctrlType)
    {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        g_running.store(false);
        return TRUE;
    default:
        return FALSE;
    }
}

static bool SendAll(SOCKET s, const char* data, int len)
{
    while (len > 0 && g_running.load())
    {
        int sent = ::send(s, data, len, 0);
        if (sent <= 0) return false;
        data += sent;
        len -= sent;
    }
    return (len == 0);
}

static SOCKET ConnectLoop(const char* host, uint16_t port, int retryMs)
{
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, host, &addr.sin_addr);

    while (g_running.load())
    {
        SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET)
        {
            Sleep(retryMs);
            continue;
        }

        if (::connect(s, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            closesocket(s);
            std::cerr << "Waiting for server " << host << ":" << port << " ...\n";
            Sleep(retryMs);
            continue;
        }

        std::cerr << "Connected to " << host << ":" << port << "\n";
        return s;
    }

    return INVALID_SOCKET;
}

static void ReceiverThread(SOCKET s, std::atomic<bool>& connected)
{
    std::string buffer;
    buffer.reserve(4096);
    std::vector<char> tmp(1024);

    while (g_running.load() && connected.load())
    {
        int n = ::recv(s, tmp.data(), (int)tmp.size(), 0);
        if (n <= 0)
        {
            connected.store(false);
            break;
        }

        buffer.append(tmp.data(), tmp.data() + n);

        // Print complete lines
        for (;;)
        {
            size_t nl = buffer.find('\n');
            if (nl == std::string::npos) break;

            std::string line = buffer.substr(0, nl + 1);
            buffer.erase(0, nl + 1);

            // Trim CR/LF for printing
            while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();

            if (!line.empty())
                std::cout << "[server] " << line << "\n";
        }

        if (buffer.size() > (1u << 20))
            buffer.clear();
    }
}

int main(int argc, char** argv)
{
    const char* host = "127.0.0.1";
    uint16_t port = 7777;
    if (argc >= 2) port = (uint16_t)std::stoi(argv[1]);

    SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);

    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }

    std::cerr << "Type commands. Use /quit or /exit to stop. Ctrl+C also works.\n";

    while (g_running.load())
    {
        SOCKET s = ConnectLoop(host, port, 500);
        if (!g_running.load() || s == INVALID_SOCKET) break;

        std::atomic<bool> connected{ true };
        std::thread rx(ReceiverThread, s, std::ref(connected));

        std::string line;
        while (g_running.load() && connected.load())
        {
            if (!std::getline(std::cin, line))
            {
                g_running.store(false);
                break;
            }

            if (line == "/quit" || line == "/exit")
            {
                g_running.store(false);
                break;
            }

            line.push_back('\n');
            if (!SendAll(s, line.data(), (int)line.size()))
            {
                connected.store(false);
                break;
            }
        }

        // Cleanup this connection
        shutdown(s, SD_BOTH);
        closesocket(s);

        connected.store(false);
        if (rx.joinable()) rx.join();

        if (g_running.load())
            std::cerr << "Disconnected. Reconnecting...\n";
    }

    WSACleanup();
    return 0;
}
