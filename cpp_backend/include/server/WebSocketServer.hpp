#pragma once
// A deliberately small HTTP/1.1 server on plain sockets (Winsock on Windows,
// BSD sockets elsewhere) — no libraries. Two jobs:
//   1. serve the static web UI (web_frontend/),
//   2. expose the simulation REST API under /api/... .
// (The file is named WebSocketServer for historical reasons; it speaks HTTP.)

#include <cstddef>
#include <functional>
#include <map>
#include <string>

class HttpServer {
public:
    using Params = std::map<std::string, std::string>;          // ?a=b&c=d
    // `body` is the request body, empty for GET. It exists so a map can be
    // uploaded: an OSM extract is megabytes, which is far past what a query
    // string can carry.
    using ApiHandler = std::function<std::string(const std::string& path,
                                                 const Params& params,
                                                 const std::string& body)>;

    // Requests larger than this are refused rather than buffered. A city-sized
    // GeoJSON is a few MB; anything much past that is a mistake or an attack,
    // and this server reads the whole body into memory.
    static constexpr std::size_t kMaxBodyBytes = 64u * 1024u * 1024u;

    HttpServer(int port, std::string document_root, ApiHandler api_handler);

    // Blocking accept loop; each connection is served on its own thread.
    void run();

private:
    void handle_client(long long client_handle);
    static std::string url_decode(const std::string& in);
    static Params parse_query(const std::string& query);
    static std::string content_type_for(const std::string& path);

    int port_;
    std::string document_root_;
    ApiHandler api_handler_;
};
