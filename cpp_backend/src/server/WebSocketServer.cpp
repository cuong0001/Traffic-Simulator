#include "server/WebSocketServer.hpp"

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <sys/socket.h>
  #include <unistd.h>
#endif

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

namespace {

constexpr int kRecvTimeoutMs = 15000;
constexpr int kMaxKeepAliveRequests = 10000;

// Small portability layer: Winsock vs BSD sockets.
#ifdef _WIN32
using socket_t = SOCKET;
constexpr socket_t kInvalidSocket = INVALID_SOCKET;
void close_socket(socket_t s) { ::closesocket(s); }

struct WinsockInit {
    WinsockInit() {
        WSADATA data;
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) std::cerr << "WSAStartup failed\n";
    }
    ~WinsockInit() { WSACleanup(); }
};
#else
using socket_t = int;
constexpr socket_t kInvalidSocket = -1;
void close_socket(socket_t s) { ::close(s); }
#endif

void send_all(socket_t fd, const std::string& data) {
    std::size_t sent = 0;
    while (sent < data.size()) {
#ifdef _WIN32
        const int n = ::send(fd, data.data() + sent, static_cast<int>(data.size() - sent), 0);
#else
        const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, 0);
#endif
        if (n <= 0) return;
        sent += static_cast<std::size_t>(n);
    }
}

void send_response(socket_t fd, const std::string& status, const std::string& content_type,
                   const std::string& body, bool keep_alive) {
    std::string response;
    response.reserve(body.size() + 192);
    response += "HTTP/1.1 ";
    response += status;
    response += "\r\nContent-Type: ";
    response += content_type;
    response += "\r\nContent-Length: ";
    response += std::to_string(body.size());
    response += "\r\nCache-Control: no-store\r\nConnection: ";
    response += keep_alive ? "keep-alive\r\nKeep-Alive: timeout=15\r\n\r\n" : "close\r\n\r\n";
    response += body;
    send_all(fd, response);
}

}  // namespace

HttpServer::HttpServer(int port, std::string document_root, ApiHandler api_handler)
    : port_(port),
      document_root_(std::move(document_root)),
      api_handler_(std::move(api_handler)) {}

std::string HttpServer::url_decode(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '%' && i + 2 < in.size()) {
            const std::string hex = in.substr(i + 1, 2);
            out += static_cast<char>(std::strtol(hex.c_str(), nullptr, 16));
            i += 2;
        } else if (in[i] == '+') {
            out += ' ';
        } else {
            out += in[i];
        }
    }
    return out;
}

HttpServer::Params HttpServer::parse_query(const std::string& query) {
    Params params;
    std::stringstream ss(query);
    std::string pair;
    while (std::getline(ss, pair, '&')) {
        const auto eq = pair.find('=');
        if (eq == std::string::npos) {
            params[url_decode(pair)] = "";
        } else {
            params[url_decode(pair.substr(0, eq))] = url_decode(pair.substr(eq + 1));
        }
    }
    return params;
}

std::string HttpServer::content_type_for(const std::string& path) {
    auto ends_with = [&](const char* suffix) {
        const std::size_t n = std::strlen(suffix);
        return path.size() >= n && path.compare(path.size() - n, n, suffix) == 0;
    };
    if (ends_with(".html")) return "text/html; charset=utf-8";
    if (ends_with(".css")) return "text/css; charset=utf-8";
    if (ends_with(".js")) return "application/javascript; charset=utf-8";
    if (ends_with(".json")) return "application/json";
    if (ends_with(".png")) return "image/png";
    if (ends_with(".jpg") || ends_with(".jpeg")) return "image/jpeg";
    if (ends_with(".svg")) return "image/svg+xml";
    if (ends_with(".ico")) return "image/x-icon";
    return "application/octet-stream";
}

void HttpServer::run() {
#ifdef _WIN32
    static WinsockInit winsock_init;  // once per process
#endif
    const socket_t server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == kInvalidSocket) {
        std::cerr << "socket() failed\n";
        return;
    }
    const int enable = 1;
    ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char*>(&enable), sizeof(enable));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        std::cerr << "bind() failed - is port " << port_ << " already in use?\n";
        close_socket(server_fd);
        return;
    }
    if (::listen(server_fd, 32) < 0) {
        std::cerr << "listen() failed\n";
        close_socket(server_fd);
        return;
    }

    std::cout << "  Urban Traffic Simulator running.\n"
              << "  Open http://localhost:" << port_ << " in your browser.\n";

    while (true) {
        const socket_t client_fd = ::accept(server_fd, nullptr, nullptr);
        if (client_fd == kInvalidSocket) continue;

        const int nodelay = 1;
        ::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY,
                     reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
#ifdef _WIN32
        const DWORD timeout_ms = kRecvTimeoutMs;
        ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO,
                     reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms));
#else
        timeval tv{};
        tv.tv_sec = kRecvTimeoutMs / 1000;
        tv.tv_usec = (kRecvTimeoutMs % 1000) * 1000;
        ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
        std::thread([this, client_fd] {
            handle_client(static_cast<long long>(client_fd));
            close_socket(client_fd);
        }).detach();
    }
}

void HttpServer::handle_client(long long client_handle) {
    const socket_t client_fd = static_cast<socket_t>(client_handle);
    char buffer[8192];

    for (int served = 0; served < kMaxKeepAliveRequests; ++served) {
#ifdef _WIN32
        const int n = ::recv(client_fd, buffer, static_cast<int>(sizeof(buffer) - 1), 0);
#else
        const ssize_t n = ::recv(client_fd, buffer, sizeof(buffer) - 1, 0);
#endif
        if (n <= 0) return;
        buffer[n] = '\0';

        std::istringstream request(buffer);
        std::string method, target, version;
        request >> method >> target >> version;
        if (method.empty() || target.empty()) {
            send_response(client_fd, "400 Bad Request", "text/plain", "bad request", false);
            return;
        }

        std::string path = target;
        std::string query;
        const auto qm = target.find('?');
        if (qm != std::string::npos) {
            path = target.substr(0, qm);
            query = target.substr(qm + 1);
        }

        bool client_wants_close = false;
        {
            std::string head(buffer, static_cast<std::size_t>(n));
            for (auto& c : head) c = static_cast<char>(::tolower(c));
            const auto pos = head.find("connection:");
            if (pos != std::string::npos && head.find("close", pos) < head.find("\r\n", pos))
                client_wants_close = true;
        }
        const bool keep_alive = !client_wants_close && served + 1 < kMaxKeepAliveRequests &&
                                version.find("1.1") != std::string::npos;

        // ---- API ----
        if (path.rfind("/api/", 0) == 0) {
            try {
                const std::string body = api_handler_(path, parse_query(query));
                send_response(client_fd, "200 OK", "application/json", body, keep_alive);
            } catch (const std::exception& e) {
                send_response(client_fd, "400 Bad Request", "application/json",
                              std::string("{\"error\":\"") + e.what() + "\"}", keep_alive);
            }
            if (!keep_alive) return;
            continue;
        }

        // ---- static files ----
        // The UI entry point lives in web_frontend/html/ and pulls assets from
        // ../css and ../js, so the document root is web_frontend and "/" serves
        // html/index.html.
        if (path == "/") path = "/html/index.html";
        if (path.find("..") != std::string::npos) {
            send_response(client_fd, "403 Forbidden", "text/plain", "forbidden", false);
            return;
        }
        const std::string file_path = document_root_ + path;
        std::ifstream file(file_path, std::ios::binary);
        // Use is_open(): on MinGW, operator! reports "good" for a missing file
        // until an actual read, which would 200-serve an empty body.
        if (!file.is_open()) {
            send_response(client_fd, "404 Not Found", "text/plain", "not found: " + path,
                          keep_alive);
            if (!keep_alive) return;
            continue;
        }
        std::ostringstream content;
        content << file.rdbuf();
        send_response(client_fd, "200 OK", content_type_for(file_path), content.str(), keep_alive);
        if (!keep_alive) return;
    }
}
