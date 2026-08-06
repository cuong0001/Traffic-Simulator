# Module 09 — Server and Entrypoint

**Files covered:**
- [`cpp_backend/include/server/WebSocketServer.hpp`](../cpp_backend/include/server/WebSocketServer.hpp) (43 lines)
- [`cpp_backend/src/server/WebSocketServer.cpp`](../cpp_backend/src/server/WebSocketServer.cpp) (321 lines)
- [`cpp_backend/src/main.cpp`](../cpp_backend/src/main.cpp) (323 lines)

Before reading a single line: the class name lies. `WebSocketServer.hpp` declares a
class called `HttpServer`, and the file's own top comment says so outright —
"The file is named WebSocketServer for historical reasons; it speaks HTTP." There is
no WebSocket upgrade handshake anywhere in this file, no `Sec-WebSocket-Accept`
header, no frame parsing. This is a small, hand-rolled HTTP/1.1 server on raw
sockets — Winsock on Windows, BSD sockets everywhere else — with zero third-party
dependencies. `CMakeLists.txt` links `ws2_32` on Windows specifically for this file
(see [`learn/01-orientation.md`](01-orientation.md), lines 63–68), and now you know
concretely what that link is for: Winsock's socket API, not anything WebSocket-protocol-specific.

Keep that straight for the rest of this module: "the server" below always means
this plain-HTTP server, and every time the frontend talks to the backend, it's
issuing an HTTP request (`fetch`), not opening a socket that stays open.

---

## `cpp_backend/include/server/WebSocketServer.hpp`

### Lines 1–6: the file banner

```cpp
#pragma once
// A deliberately small HTTP/1.1 server on plain sockets (Winsock on Windows,
// BSD sockets elsewhere) — no libraries. Two jobs:
//   1. serve the static web UI (web_frontend/),
//   2. expose the simulation REST API under /api/... .
// (The file is named WebSocketServer for historical reasons; it speaks HTTP.)
```

**Meaning:** `#pragma once` is a non-standard but universally supported include
guard — the compiler tracks which files it's already pulled in and skips
re-inclusion, avoiding the classic double-definition problem without the
boilerplate of `#ifndef`/`#define`/`#endif`. The comment block states the file's
two jobs explicitly and confesses the naming mismatch up front.

**Why:** a hand-rolled server (no `Boost.Asio`, no `cpp-httplib`, no libuv) is a
deliberate scope decision — the project has "no dependencies" as a constraint
mentioned elsewhere in the codebase (see `main.cpp`'s `/api/importMap` comment
below), and a two-job HTTP server (static files + a JSON API) is small enough to
write by hand without inheriting a large library just for socket plumbing.

**Usage:** Every other file in this module either implements this class
(`WebSocketServer.cpp`) or constructs and runs it (`main.cpp`).

### Lines 8–11: includes

```cpp
#include <cstddef>
#include <functional>
#include <map>
#include <string>
```

**Meaning:** `<cstddef>` for `std::size_t`, `<functional>` for `std::function`,
`<map>` for the `Params` alias, `<string>` for obvious reasons. Notice what's
*not* here: no `<thread>`, no socket headers. Those are implementation details,
confined to the `.cpp` file — the header only needs to declare the public shape
of the class, not how it's built.

**Why:** keeping platform-specific socket headers (`winsock2.h` vs.
`sys/socket.h`) out of the header means anything that `#include`s
`WebSocketServer.hpp` (like `main.cpp`) doesn't have to deal with `WIN32_LEAN_AND_MEAN`
or Winsock initialization order — those concerns are fully encapsulated in the
`.cpp` file.

**Usage:** This is ordinary interface/implementation separation, the same
principle you'd expect from any header in `cpp_backend/include/`.

### Lines 13–21: the class opening and its two public aliases

```cpp
class HttpServer {
public:
    using Params = std::map<std::string, std::string>;          // ?a=b&c=d
    // `body` is the request body, empty for GET. It exists so a map can be
    // uploaded: an OSM extract is megabytes, which is far past what a query
    // string can carry.
    using ApiHandler = std::function<std::string(const std::string& path,
                                                 const Params& params,
                                                 const std::string& body)>;
```

**Meaning:** `Params` is a type alias for a string-to-string map — the decoded
query-string key/value pairs from a URL like `/api/spawn?from=A&to=B`.
`ApiHandler` is a `std::function` alias for "a callable that takes a path, the
parsed query params, and a raw request body string, and returns a JSON string."
This is the server's entire contract with whoever embeds it: give me a function
matching this signature, and I'll call it for every request under `/api/`.

**Why:** using `std::function` instead of a fixed free-function pointer or an
abstract base class with a virtual `handle()` method means the caller can hand
in a *lambda that captures state* — which is exactly what `main.cpp` does (its
`api` lambda captures `sim` by reference). This is the mechanism by which the
generic, reusable `HttpServer` class gets wired to the very specific
`TrafficSimulator` singleton without the server needing to know
`TrafficSimulator` exists at all — a clean inversion of control. The comment on
`body` explains why the API isn't GET-only despite every endpoint in `main.cpp`
being read via query string: `/api/importMap` needs to carry a multi-megabyte
GeoJSON payload, and query strings have practical length limits imposed by
browsers/servers that a request body doesn't.

**Usage:** `main.cpp` builds a single `ApiHandler`-shaped lambda (its `api`
variable, covered below) and passes it into the `HttpServer` constructor. The
server never calls into `TrafficSimulator` directly — only through this
function object.

### Lines 23–26: the body size cap

```cpp
    // Requests larger than this are refused rather than buffered. A city-sized
    // GeoJSON is a few MB; anything much past that is a mistake or an attack,
    // and this server reads the whole body into memory.
    static constexpr std::size_t kMaxBodyBytes = 64u * 1024u * 1024u;
```

**Meaning:** A `static constexpr` — a compile-time constant belonging to the
class itself, not to any instance — set to 64 MiB (`64 * 1024 * 1024` bytes).

**Why:** the comment states the real constraint plainly: this server reads an
entire request body into a single `std::string` in memory before handing it to
the API handler (there's no streaming). Without a cap, a malicious or broken
client could send a body without end and grow that buffer until the process
runs out of memory — a classic unbounded-allocation denial-of-service vector.
64 MiB is generous headroom over "a city-sized GeoJSON," which the comment says
is typically a few MB.

**Usage:** Enforced twice in `WebSocketServer.cpp::handle_client` — once against
the declared `Content-Length` header (reject before reading), and once as a
safety net while still accumulating header bytes if a client never sends a
blank line to terminate the headers. Both are covered below.

### Lines 28–31: the constructor and `run()`

```cpp
    HttpServer(int port, std::string document_root, ApiHandler api_handler);

    // Blocking accept loop; each connection is served on its own thread.
    void run();
```

**Meaning:** The constructor takes the TCP port to listen on, the filesystem
directory to serve static files from, and the `ApiHandler` callback. `run()`
takes no arguments and returns `void` — the comment tells you upfront that it
never returns under normal operation: it's a blocking loop that spawns a new
thread per accepted connection.

**Why:** "one thread per connection" is the simplest possible concurrency model
for a server — no event loop, no `epoll`/`IOCP` abstraction to hand-roll. It's a
reasonable choice for a server whose expected client count is "one browser tab,
maybe two," not a production web service handling thousands of concurrent
connections. This is also *why* `find_package(Threads REQUIRED)` shows up in
`CMakeLists.txt` (see [`learn/01-orientation.md`](01-orientation.md)) even though
the simulation itself runs on a "single-threaded tick loop"
(see [`learn/07-simulation-core.md`](07-simulation-core.md)) — the threading
requirement comes from here, not from the simulation engine.

**Usage:** `main.cpp` constructs one `HttpServer` and calls `server.run()` as
the very last statement in `main`, blocking forever (or until the process is
killed) — see the `main.cpp` walkthrough below.

### Lines 33–42: private members

```cpp
private:
    void handle_client(long long client_handle);
    static std::string url_decode(const std::string& in);
    static Params parse_query(const std::string& query);
    static std::string content_type_for(const std::string& path);

    int port_;
    std::string document_root_;
    ApiHandler api_handler_;
};
```

**Meaning:** `handle_client` is the per-connection worker, taking a
`long long client_handle` rather than a platform socket type. The three
`static` helpers are pure functions with no dependency on instance state:
percent-decoding a URL component, splitting a query string into `Params`, and
mapping a file extension to a MIME type string. The three data members mirror
the three constructor parameters exactly.

**Why:** `client_handle` being a `long long` instead of the platform-native
socket type (`SOCKET` on Windows, `int` on POSIX) is a deliberate leak-avoidance
choice — it lets the *header* stay platform-agnostic (no `#ifdef _WIN32`
needed here) while the `.cpp` file casts it back to the real socket type
internally. Making the three helpers `static` (rather than requiring a `this`)
signals they're stateless utilities — you could unit test `url_decode` without
constructing an `HttpServer` at all.

**Usage:** `handle_client` is spawned once per accepted connection inside
`run()`'s accept loop (`.cpp` file, covered next); the three static helpers are
called from inside `handle_client` and from `content_type_for`'s own caller.

**Check yourself:**
- Why does `ApiHandler` take the raw `body` string as a separate parameter instead of folding it into `Params`?
- What would break if `client_handle` were typed as `int` in the header instead of `long long`?
- The class is literally named `HttpServer` in code but the file is `WebSocketServer.hpp`. What protocol does it actually speak, and what evidence in the header proves it?

---

## `cpp_backend/src/server/WebSocketServer.cpp`

### Lines 1–24: includes and the platform split

```cpp
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
```

**Meaning:** `#ifdef _WIN32` branches the socket headers: Winsock2
(`winsock2.h`, `ws2tcpip.h`) on Windows, Berkeley sockets (`arpa/inet.h`,
`netinet/in.h`, `netinet/tcp.h`, `sys/socket.h`, `unistd.h`) elsewhere.
`WIN32_LEAN_AND_MEAN` is defined (guarded by its own `#ifndef` so it's not
redefined if something upstream already set it) before including `winsock2.h`
to strip rarely-needed Windows headers (like most of the GUI API) out of the
massive `windows.h` that `winsock2.h` pulls in transitively, shortening compile
times and avoiding macro collisions (notably the infamous `min`/`max` macros).
`<thread>` is the C++ standard library's thread support, used for the
per-connection thread. `<fstream>` is for reading static files off disk.

**Why:** this is the concrete proof the class does raw sockets, not an
HTTP/WebSocket library — there's no `#include <websocketpp/...>` or similar
anywhere. Everything is built on the OS socket API directly, which is also why
the platform split has to happen this low in the file: Winsock and POSIX
sockets are similar in shape (`socket()`, `bind()`, `listen()`, `accept()`,
`recv()`, `send()`) but differ in header layout, error handling conventions, and
a few type names.

**Usage:** Every socket call for the rest of the file resolves to one of these
two header sets depending on platform, via the type aliases defined next.

### Lines 25–47: the socket-type portability layer

```cpp
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
```

**Meaning:** An anonymous namespace (`namespace { ... }`) gives everything
inside it internal linkage — none of these names are visible outside this
translation unit, avoiding symbol clashes at link time. `kRecvTimeoutMs` (15000
ms = 15 s) and `kMaxKeepAliveRequests` (10000) are the two tunables governing
how long a connection can sit idle and how many requests one kept-alive TCP
connection will serve before the server forces it closed. On Windows,
`socket_t` aliases `SOCKET` (an opaque handle type), `kInvalidSocket` aliases
`INVALID_SOCKET`, and `close_socket` wraps `closesocket()`. `WinsockInit` is an
RAII (Resource Acquisition Is Initialization) struct: its constructor calls
`WSAStartup` (mandatory on Windows before any socket call will work) and its
destructor calls `WSACleanup`. On POSIX, `socket_t` is a plain `int` (the
traditional file-descriptor type), `kInvalidSocket` is `-1`, and
`close_socket` wraps `close()`.

**Why:** `WinsockInit` being a class with constructor/destructor (rather than a
manual `WSAStartup()`/`WSACleanup()` call pair) means it's exception-safe and
impossible to forget to clean up — this is the RAII idiom you've likely already
seen elsewhere in this codebase for lock guards; here it's applied to a global
OS resource instead of a mutex. POSIX has no equivalent startup step, hence the
`#else` branch defines nothing analogous — POSIX sockets just work once you
call `socket()`.

**Usage:** `WinsockInit` gets instantiated exactly once, as a `static` local
inside `run()` (see below) — "once per process" per that comment. `socket_t`,
`kInvalidSocket`, and `close_socket` are used throughout the rest of the file to
stay platform-neutral in the code that follows.

### Lines 49–60: `send_all`

```cpp
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
```

**Meaning:** `send()` is not guaranteed to write an entire buffer in one call —
it can return having sent fewer bytes than requested, particularly for large
payloads or a congested socket. This loop tracks how many bytes have been
confirmed sent (`sent`) and keeps calling `::send()` with the remaining tail of
`data` until either everything is sent or `send()` returns `<= 0` (an error or
the connection closing), in which case the function gives up silently.

**Why:** this is a textbook example of exactly the kind of "accept-loop/socket
code" the task brief calls out as easy to get subtly wrong — forgetting this
loop and calling `send()` once, assuming it always writes the whole buffer, is
a common bug that only shows up under load (large responses, slow networks) and
is easy to miss in casual local testing where the OS send buffer usually
absorbs everything in one call. Silently returning on `n <= 0` means a failed
send never throws or logs — the caller (`send_response`, next) has no way to
know delivery failed, which is a real, if minor, blind spot: a client that
disconnects mid-response just gets a truncated write with no diagnostic. That's
an acceptable tradeoff for a tool this size, but it's worth naming as a rough
edge rather than glossing over it.

**Usage:** Every response the server ever sends — static file bytes, JSON API
replies, error pages — goes through `send_all` via `send_response` below.

### Lines 62–76: `send_response`

```cpp
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
```

**Meaning:** Builds a complete HTTP/1.1 response as one string, by hand: status
line, `Content-Type`, `Content-Length` (computed from the actual body size, not
trusted from anywhere), `Cache-Control: no-store` (always — every response
tells the browser never to cache it), and a `Connection` header that's either
`keep-alive` (with a `Keep-Alive: timeout=15` hint matching `kRecvTimeoutMs`) or
`close`, followed by a blank line (`\r\n\r\n`, the HTTP header/body separator)
and the body itself. `response.reserve(...)` pre-allocates the string's buffer
to avoid repeated reallocation as `+=` grows it.

**Why:** `Cache-Control: no-store` unconditionally makes sense given this
server's payloads: `/api/state` and `/api/stateBin` are live simulation
snapshots that change every tick — caching them would show the browser a stale
world. Building the response by string concatenation instead of using any HTTP
library is consistent with the "no libraries" design already established; the
manual `\r\n` line endings are required by the HTTP spec (bare `\n` is not
technically valid, though most real servers tolerate it — this one does it
correctly).

**Usage:** Called from three places in `handle_client`: once for API replies,
once for static file replies, and once each for the various error paths (400,
403, 404, 413) covered below.

### Lines 80–83: the constructor definition

```cpp
HttpServer::HttpServer(int port, std::string document_root, ApiHandler api_handler)
    : port_(port),
      document_root_(std::move(document_root)),
      api_handler_(std::move(api_handler)) {}
```

**Meaning:** A member initializer list assigning each constructor parameter to
its matching data member. `document_root` and `api_handler` are taken by value
and then `std::move`-d into the members — avoiding a copy of the string and the
`std::function` (which can itself own captured state, e.g. a lambda closure)
when the caller passes an rvalue or is fine giving up ownership.

**Why:** "take by value, then move" is the standard modern-C++ idiom for
constructors that unconditionally store a copy of their argument — it lets the
compiler elide the copy entirely when the caller passes a temporary, while
still working correctly (with exactly one copy) when the caller passes an
lvalue they still need afterward.

**Usage:** `main.cpp` calls this constructor once: `HttpServer server(port,
"web_frontend", api);` — covered in the `main.cpp` section.

### Lines 85–100: `url_decode`

```cpp
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
```

**Meaning:** Implements percent-decoding for URL query components: a `%XX`
triplet (where `XX` is two hex digits) becomes the byte with that hex value
(parsed via `std::strtol` base 16), a literal `+` becomes a space (the
`application/x-www-form-urlencoded` convention, still honored here even though
these are GET query strings, not form bodies), and every other character
passes through unchanged.

**Why:** the bounds check `i + 2 < in.size()` (strictly less-than, not
less-or-equal) is worth scrutinizing closely, since this is exactly the kind of
low-level parsing code the task brief flags as bug-prone. If `in` ends with a
well-formed `%XX` sequence as its *last three characters*, `i` points at `%`,
and `in.size() - 1` is the index of the last hex digit — so `i + 2 == in.size() - 1`,
meaning `i + 2 < in.size()` holds true. So a trailing `%XX` at the very end
of the string *is* handled correctly; the check is really guarding against a
truncated `%X` or a bare trailing `%` from indexing past the end of the string
in `substr`. This is correct, just worth tracing carefully rather than
assuming.

**Usage:** Called from `parse_query` (next) on both the key and value of every
`key=value` pair in a query string — e.g. turning `from=Junction%20A` into
`from = "Junction A"`.

### Lines 102–115: `parse_query`

```cpp
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
```

**Meaning:** Splits the query string on `&` using `std::getline` with a custom
delimiter (a lesser-known overload — `getline` isn't only for reading lines
from files, it splits any stream on any single-character delimiter). For each
`key=value` chunk, it looks for the first `=`; if there isn't one, the whole
chunk becomes a key with an empty value (`?flag` with no `=value`); otherwise
it splits at the `=` and decodes both halves.

**Why:** using `Params` (a `std::map`) rather than a `std::multimap` or vector
of pairs means a repeated key silently overwrites the previous value — this
server's API never relies on repeated keys, so that's a fine simplification,
but it's worth knowing it's there for the day someone tries `?id=1&id=2`.

**Usage:** Called once per request from `handle_client`, right before
dispatching to `api_handler_` — the `Params` map it returns is exactly the
second argument of `ApiHandler`.

### Lines 117–131: `content_type_for`

```cpp
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
```

**Meaning:** A local lambda `ends_with` checks a suffix match safely (guarding
`path.size() >= n` before calling `compare` with a negative-would-be offset), then
a chain of `if`s maps known extensions to MIME types, falling back to the
generic binary type `application/octet-stream` for anything unrecognized.

**Why:** browsers use the `Content-Type` header to decide how to interpret a
response — serving `.js` as `application/octet-stream` would make the browser
offer to download it instead of executing it as a module, breaking the whole
frontend. This is a hardcoded, small table rather than reading from `/etc/mime.types`
or a library, consistent with the file's "no libraries" scope.

**Usage:** Called once per static-file response, in the static-file branch of
`handle_client`, to set the `Content-Type` header on the served file.

### Lines 133–163: `run()` — socket setup

```cpp
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
```

**Meaning:** `static WinsockInit winsock_init;` is a function-local static —
initialized exactly once, the first time `run()` is called, and destroyed at
process exit; this is what "once per process" (the comment) means concretely,
and it's a common, correct C++ idiom for lazy one-time global setup. `::socket(AF_INET, SOCK_STREAM, 0)`
creates an IPv4 (`AF_INET`), TCP (`SOCK_STREAM`) socket. `SO_REUSEADDR` is set
so the server can rebind to the same port immediately after a restart, without
waiting out the OS's `TIME_WAIT` teardown period. `sockaddr_in address{}` is
zero-initialized (the `{}`), then filled in: `AF_INET` again, `INADDR_ANY`
(listen on all local network interfaces, not just `127.0.0.1`), and the port,
converted from host byte order to network byte order via `htons` (the port
field is 16-bit; `htonl`/`htons` handle endianness portably). `bind()`
associates the socket with that address/port; `listen(server_fd, 32)` puts the
socket into listening mode with a backlog of 32 — the OS will queue up to 32
pending connections before starting to refuse new ones while the accept loop
is busy.

**Why:** each failure path (`socket()`, `bind()`, `listen()`) prints a specific
diagnostic and returns early rather than continuing in a broken state — the
`bind()` failure message even guesses the most likely cause ("is port ...
already in use?"), which is good defensive UX for a tool a developer runs
directly from a terminal. `INADDR_ANY` (rather than binding only to loopback)
means the server is reachable from other machines on the same network, not
just `localhost` — worth knowing if you're running this on a shared network,
since there's no authentication anywhere in this file (see the `/api/` dispatch
below — no token, no login).

**Usage:** This is the one-time setup that happens before the server can accept
any connections at all; if any step fails, `run()` returns and `main()`'s call
to `server.run()` returns too, ending the program (see `main.cpp`'s last line).

### Lines 162–186: the accept loop

```cpp
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
```

**Meaning:** After printing the "server is up" banner (this is the message
you'll see in the terminal when you launch `traffic_simulator`), `while (true)`
begins an infinite loop. `::accept()` blocks until a client connects, returning
a *new* socket (`client_fd`) representing that one connection — `server_fd`
itself never carries data, it only produces new connection sockets.
`if (client_fd == kInvalidSocket) continue;` — if `accept()` fails, skip this
iteration and try again rather than crashing. `TCP_NODELAY` disables Nagle's
algorithm (which normally batches small writes to reduce packet count at the
cost of latency) — appropriate for a server sending many small JSON responses
where per-request latency matters more than packet efficiency. The
`SO_RCVTIMEO` block (platform-split, `DWORD` milliseconds on Windows vs.
`timeval` seconds+microseconds on POSIX) sets a receive timeout of
`kRecvTimeoutMs` (15 s) on the *client* socket — a `recv()` call that gets no
data within 15 seconds will fail rather than block forever. Finally,
`std::thread([this, client_fd] { ... }).detach()` spawns a new OS thread per
connection, capturing `this` (so the thread can call `handle_client`, a member
function) and `client_fd` by value; the thread's body calls `handle_client`
then closes the socket; `.detach()` immediately severs the `std::thread`
object's ownership of the underlying OS thread, letting it run independently
and clean itself up when it finishes — there's no `join()` anywhere, so `run()`
never waits for any connection to finish before accepting the next one.

**Why:** this is the concurrency model the task brief specifically asks about:
the server accepts connections on the *main thread* (inside `run()`'s loop),
but each individual connection's request/response handling happens on its own
*detached* thread. That means an arbitrary number of browser requests can be
in flight simultaneously, each blocked on its own `recv()`/`send()` calls,
without blocking the accept loop from picking up the next connection.
`SO_RCVTIMEO` exists specifically to bound how long a misbehaving or idle
client can tie up one of these threads — without it, a client that connects and
never sends anything would leak a thread forever. The `continue` on a failed
`accept()` (rather than `return`, ending `run()` entirely) means one transient
accept failure doesn't take the whole server down — a defensible choice, though
it does mean a persistently failing `accept()` (e.g. file-descriptor
exhaustion) would spin this loop indefinitely without diagnostic output, since
there's no logging on this path. That's a real rough edge worth naming, in the
same spirit as the "flag known rough edges honestly" rule for this course.

**Usage:** This is the loop `main.cpp` invokes forever via `server.run()`. Every
`fetch()` call from `web_frontend/js/api.js` (covered below) results in exactly
one pass through this loop: one `accept()`, one detached thread, one call to
`handle_client`.

### Lines 189–205: `header_value`

```cpp
static std::string header_value(const std::string& head, const std::string& name) {
    std::string lower = head;
    for (auto& c : lower) c = static_cast<char>(::tolower(c));
    const auto at = lower.find("\r\n" + name + ":");
    if (at == std::string::npos) return "";
    const auto start = at + name.size() + 3;
    const auto end = lower.find("\r\n", start);
    if (end == std::string::npos) return "";
    std::string v = head.substr(start, end - start);
    const auto b = v.find_first_not_of(" \t");
    if (b == std::string::npos) return "";
    const auto e = v.find_last_not_of(" \t");
    return v.substr(b, e - b + 1);
}
```

**Meaning:** A free function (not a member — declared `static` at file scope,
giving it internal linkage, the free-function equivalent of the anonymous
namespace used earlier) that extracts one header's value from the raw request
head text. It lowercases a *copy* of the whole head (`lower`), since header
names are case-insensitive per HTTP spec but the search needs an exact string
match; it searches for `"\r\n" + name + ":"` (the header name preceded by a
line break and followed by a colon, ensuring it matches a header line and not,
say, a substring inside another header's value). If found, it computes `start`
(just past the `\r\n` + name + `:`) and finds the next `\r\n` to get `end`, then
extracts that span *from the original, non-lowercased `head`* (`v = head.substr(...)`)
so the returned value preserves its original casing. The final two lines trim
leading/trailing spaces and tabs from the value using `find_first_not_of`/`find_last_not_of`.

**Why:** searching on a lowercased copy while extracting from the original is
the detail worth noticing here — get this backwards (lowercase everything, then
return substrings of the lowercased string) and header values like a
`Content-Type` with meaningful casing would come back mangled. This function
is small, hand-rolled string parsing again — no HTTP parsing library — consistent
with the rest of the file, and it's exactly the kind of code that's easy to get
subtly wrong (as the task brief warns), so it rewards a careful read rather
than a skim.

**Usage:** Called twice in `handle_client`: once for `Content-Length` (to know
how many body bytes to expect) and once for `Connection` (to decide
keep-alive vs. close).

### Lines 207–227: `handle_client` — reading the header

```cpp
void HttpServer::handle_client(long long client_handle) {
    const socket_t client_fd = static_cast<socket_t>(client_handle);
    char chunk[8192];
    // Carried across loop iterations: one recv() on a keep-alive connection can
    // pull in the front of the NEXT request, and discarding that would stall.
    std::string pending;

    for (int served = 0; served < kMaxKeepAliveRequests; ++served) {
        // ---- read until the head is complete ------------------------------
        std::size_t head_end = pending.find("\r\n\r\n");
        while (head_end == std::string::npos) {
#ifdef _WIN32
            const int n = ::recv(client_fd, chunk, static_cast<int>(sizeof(chunk)), 0);
#else
            const ssize_t n = ::recv(client_fd, chunk, sizeof(chunk), 0);
#endif
            if (n <= 0) return;
            pending.append(chunk, static_cast<std::size_t>(n));
            if (pending.size() > kMaxBodyBytes) return;   // headers with no end
            head_end = pending.find("\r\n\r\n");
        }
        const std::string head = pending.substr(0, head_end + 4);
```

**Meaning:** `client_handle` (the platform-agnostic `long long` from the
header) is cast back to `socket_t` here, inside the `.cpp` file, exactly as
anticipated in the header walkthrough. `chunk` is an 8 KiB stack buffer used
for each raw `recv()` call. `pending` is a `std::string` that persists *across
iterations of the outer `for` loop* — the comment explains why: on a
keep-alive connection, a single `recv()` can return bytes belonging to the
*next* request bundled after the current one's end, and those bytes must not
be thrown away. The `for (int served = 0; served < kMaxKeepAliveRequests; ++served)`
loop is the keep-alive loop — one iteration per HTTP request served on this
one TCP connection, capped at 10000 requests before the server forces a close
regardless of what the client wants. Inside, `pending.find("\r\n\r\n")` looks
for the blank line marking the end of HTTP headers; while it's not found yet
(`std::string::npos`), the inner `while` loop keeps calling `recv()` and
appending to `pending`. `if (n <= 0) return;` — a `recv()` returning zero or
negative means the connection closed or errored, so the whole function (and
thus the thread, since `handle_client` is the thread body) returns.
`if (pending.size() > kMaxBodyBytes) return;` is the second enforcement of the
64 MiB cap mentioned in the header — this one guards against headers that never
terminate, distinct from the `Content-Length` check that comes later for the
body. Once the loop exits, `head` is sliced out as everything up to and
including the terminating `\r\n\r\n`.

**Why:** this is the header-reading loop, and it demonstrates why TCP is a
byte-stream protocol, not a message protocol: there's no guarantee one
`recv()` call returns exactly one HTTP request, or even a whole request at
all — it might return a fragment, or (on keep-alive) a fragment plus part of
the next request. `pending` being a persistent buffer, sliced and then having
the consumed prefix erased later in the function (see below), is the standard
way to handle that. This is precisely the kind of subtle networking correctness
the task brief calls out — get the buffering wrong and either request-response
boundaries get scrambled, or performance dies from re-reading data.

**Usage:** This runs once per HTTP request received on the connection; it's
the first stage of the request pipeline that ultimately reaches either the
`/api/` dispatch or the static-file branch.

### Lines 228–236: parsing the request line

```cpp
        std::istringstream request(head);
        std::string method, target, version;
        request >> method >> target >> version;
        if (method.empty() || target.empty()) {
            send_response(client_fd, "400 Bad Request", "text/plain", "bad request", false);
            return;
        }
```

**Meaning:** An `std::istringstream` over the raw header text, with the
default `>>` whitespace-splitting behavior used to pull the first three
space-separated tokens off the first line: HTTP method (`GET`, `POST`, ...),
target (the path + optional query string), and version (`HTTP/1.1`). If either
`method` or `target` came back empty (malformed request line), the server
replies `400 Bad Request` and returns — closing the connection outright
(`false` for `keep_alive`, and no `continue`, so the function exits).

**Why:** using `istringstream`'s `>>` operator is a lightweight way to tokenize
without hand-writing a scanner — it naturally handles the "split on any
whitespace, skip repeats" behavior the HTTP request line format calls for.
Note `method` is parsed but genuinely never checked afterward — the server
doesn't distinguish `GET` from `POST` anywhere in the rest of the function; the
API dispatch and static-file logic both run for any method whose target
matches. That's worth flagging plainly: this server accepts, say, a `DELETE
/api/pause` exactly the same as `GET /api/pause`. It's a simplification that
works because this is a trusted, local tool rather than a public-facing API,
but it is not REST-correct.

**Usage:** `target` feeds directly into the path/query split a few lines down;
`version` is used later to help decide keep-alive eligibility.

### Lines 238–267: reading the body

```cpp
        // ---- read the body, when the head announced one -------------------
        // Needed for map import: an OSM extract is megabytes, far past what a
        // query string can carry, so it arrives as a POST body.
        std::size_t content_length = 0;
        {
            const std::string cl = header_value(head, "content-length");
            if (!cl.empty()) {
                try {
                    content_length = static_cast<std::size_t>(std::stoull(cl));
                } catch (const std::exception&) {
                    content_length = 0;
                }
            }
        }
        if (content_length > kMaxBodyBytes) {
            send_response(client_fd, "413 Payload Too Large", "application/json",
                          "{\"ok\":false,\"error\":\"body too large\"}", false);
            return;
        }
        while (pending.size() < head.size() + content_length) {
#ifdef _WIN32
            const int n = ::recv(client_fd, chunk, static_cast<int>(sizeof(chunk)), 0);
#else
            const ssize_t n = ::recv(client_fd, chunk, sizeof(chunk), 0);
#endif
            if (n <= 0) return;   // client vanished mid-body
            pending.append(chunk, static_cast<std::size_t>(n));
        }
        const std::string body = pending.substr(head.size(), content_length);
        pending.erase(0, head.size() + content_length);
```

**Meaning:** Looks up `Content-Length` via `header_value`; if present, parses
it with `std::stoull` (string to unsigned long long) inside a `try`/`catch` —
a malformed value falls back to `content_length = 0` rather than throwing out
of the function. If the *declared* length exceeds `kMaxBodyBytes`, the server
replies `413 Payload Too Large` immediately, before reading a single body
byte — this is the first, cheaper enforcement of the size cap (reject based on
what the client *claims* it'll send, before spending any effort receiving it).
Then a `while` loop keeps `recv()`-ing until `pending` holds at least
`head.size() + content_length` bytes total — i.e., the full head plus the full
declared body. Once enough data has accumulated, `body` is sliced out of
`pending` at the right offset/length, and `pending.erase(0, head.size() + content_length)`
removes the just-consumed request (head + body) from the front of `pending`,
leaving anything left over (the start of the *next* pipelined request) intact
for the next loop iteration.

**Why:** this two-stage size check (reject-by-header first, then only read up
to `kMaxBodyBytes` in practice) is a reasonable defense against a client lying
about `Content-Length` — a small declared value can't be circumvented to
actually stream unlimited bytes, because the loop only reads until `pending`
reaches `head.size() + content_length`, i.e. exactly what was declared, not
"whatever arrives." A client that declares a small body and then sends more
data anyway just has that extra data sit in `pending` and get parsed as the
start of the next request (or garbage, if it wasn't meant to be a new request)
— consistent with the pipelining design goal.

**Usage:** `body` feeds directly into `api_handler_` for `/api/` requests (see
below) — this is the exact mechanism `/api/importMap`, in `main.cpp`, uses to
receive a multi-megabyte GeoJSON payload.

### Lines 270–294: path/query split and keep-alive decision

```cpp
        std::string path = target;
        std::string query;
        const auto qm = target.find('?');
        if (qm != std::string::npos) {
            path = target.substr(0, qm);
            query = target.substr(qm + 1);
        }

        const std::string conn = header_value(head, "connection");
        bool client_wants_close = conn == "close" || conn == "Close" || conn == "CLOSE";
        const bool keep_alive = !client_wants_close && served + 1 < kMaxKeepAliveRequests &&
                                version.find("1.1") != std::string::npos;

        // ---- API ----
        if (path.rfind("/api/", 0) == 0) {
            try {
                const std::string reply = api_handler_(path, parse_query(query), body);
                send_response(client_fd, "200 OK", "application/json", reply, keep_alive);
            } catch (const std::exception& e) {
                send_response(client_fd, "400 Bad Request", "application/json",
                              std::string("{\"error\":\"") + e.what() + "\"}", keep_alive);
            }
            if (!keep_alive) return;
            continue;
        }
```

**Meaning:** `target.find('?')` splits `path` from `query` at the first `?`, if
any. `conn` reads the `Connection` header via `header_value`; note the
case-sensitive `==` comparisons against `"close"`, `"Close"`, `"CLOSE"` — this
is a manual, incomplete stand-in for a case-insensitive comparison (it would
miss `"cLosE"`, for instance), a small inconsistency given that `header_value`
itself already does case-insensitive *searching* for the header name, just not
for the value. `keep_alive` is computed as a conjunction: the client didn't ask
to close, there's still room under `kMaxKeepAliveRequests` for at least one
more request on this connection, and the client's declared version string
contains `"1.1"` (HTTP/1.0 has no persistent-connection-by-default behavior,
so this server only offers keep-alive to 1.1 clients). `path.rfind("/api/", 0) == 0`
is the idiomatic (if slightly obscure) way to check "does `path` start with
`/api/`" using `rfind` anchored at position 0 rather than a full search — if it
does, this is an API request: `api_handler_` (the lambda from `main.cpp`) is
invoked with the path, the parsed query params, and the body, wrapped in a
`try`/`catch` so any exception thrown out of the handler (e.g. `std::stod`
throwing on a malformed number, seen throughout `main.cpp`'s `api` lambda)
becomes a `400 Bad Request` JSON error instead of crashing the thread. On
success, the reply is sent as `200 OK` with `application/json`. Either way, if
`keep_alive` is false the function returns (closing the connection after
`close_socket` runs in the thread lambda); if true, `continue` loops back to
serve the next request on the same connection.

**Why:** wrapping the API call in `try`/`catch` is what keeps one bad request
(e.g. `/api/speed?x=notanumber`, which would throw inside `std::stod`, visible
in `main.cpp`) from taking down the thread handling that connection —
critically, since each connection is its own detached thread, an *uncaught*
exception here would `std::terminate()` the whole process, not just fail one
request. This `try`/`catch` is a load-bearing correctness boundary, not
decoration.

**Usage:** This is the exact code path every `fetch("/api/...")` call from
`web_frontend/js/api.js` hits. The `ApiHandler` signature from the header
(`path, params, body -> JSON string`) is realized here as a direct function
call — no framework routing table, just one `if` gating a lambda call.

### Lines 296–319: the static-file branch

```cpp
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
```

**Meaning:** If the request wasn't for `/api/...`, it falls through to serving
a static file. `path == "/"` is special-cased to `html/index.html` — the
comment explains the frontend's own directory layout dictates this rewrite.
`path.find("..") != std::string::npos` is a path-traversal guard: if the
requested path contains `..` anywhere, the server refuses with `403 Forbidden`
rather than risk resolving a path like `/../../etc/passwd` outside
`document_root_`. `file_path` concatenates the document root and the
(sanitized) path; `std::ifstream file(file_path, std::ios::binary)` opens it in
binary mode (important on Windows, where text mode would translate line
endings and corrupt binary assets like images). The comment explains a real,
platform-specific gotcha: on MinGW, `operator!` on a stream can report "good"
for a file that doesn't actually exist until you try to read from it — using
`is_open()` instead is the correct, portable check. On success, the entire file
is slurped into an `ostringstream` via `content << file.rdbuf()` (reading the
whole file's stream buffer at once) and sent as a `200 OK` with the MIME type
computed by `content_type_for`.

**Why:** the `..` check is a real security control, not a nicety — without it,
this server would let any client walk the filesystem outside `web_frontend/`.
It's a simple substring check rather than full path canonicalization/resolution,
which means it's not bulletproof against every conceivable encoding trick
(e.g. it operates on the string *before* `url_decode` would matter here — note
static paths are never run through `url_decode` at all, unlike query params),
but it blocks the straightforward traversal attempt. The `is_open()` comment is
a good example of the file's overall character: written by someone who's hit
real platform-specific bugs and left a note explaining exactly why the
"obvious" alternative is wrong — precisely the kind of subtlety this course
tells you not to skim past.

**Usage:** This is what serves every file under `web_frontend/` — HTML, CSS,
JS modules, sprites, `map.json`. It's how the browser gets the entire
application in the first place, before any `/api/` call is ever made.

**Check yourself:**
- Trace exactly what happens if a client sends two pipelined HTTP/1.1 requests back-to-back in a single TCP write. How does `pending` end up holding the second request's bytes without them being lost?
- Why is the `try`/`catch` around `api_handler_(...)` a correctness requirement rather than a nicety, given how connections are handled?
- The `Connection` header comparison (`conn == "close"`) is case-sensitive while the header *name* lookup in `header_value` is case-insensitive. Construct a `Connection` header value that would slip past `client_wants_close` undetected.
- What two independent mechanisms cap request body size, and which one runs first?

---

## `cpp_backend/src/main.cpp`

### Lines 1–19: file banner and includes

```cpp
// Urban Traffic Simulator — backend entry point.
//
//   traffic_simulator [map.geojson] [port]
//
// Loads a GeoJSON map, exports web_frontend/map.json for the static viewer,
// runs the simulation on a background thread, and serves the web UI + REST API
// (/api/map, /api/state, /api/spawn, /api/event, /api/pause, /api/resume,
//  /api/speed, /api/auto, /api/history/range, /api/history/at) over HTTP.
#include "simulation/TrafficSimulator.hpp"
#include "server/WebSocketServer.hpp"
#include "core/Graph.hpp"
#include "utils/MapLoader.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
```

**Meaning:** The banner documents the command-line usage
(`traffic_simulator [map.geojson] [port]`, both optional) and gives a
non-exhaustive list of the API surface. The includes pull in
`TrafficSimulator` (the Singleton engine, see
[`learn/07-simulation-core.md`](07-simulation-core.md)), `HttpServer` (this
module), `Graph` (for reading node/edge counts at startup), and `MapLoader`
(for loading GeoJSON and writing the static `map.json` fallback).

**Why:** `main.cpp` is deliberately the *only* file compiled directly into the
`traffic_simulator` executable (per `CMakeLists.txt`, see
[`learn/01-orientation.md`](01-orientation.md)) — everything it needs comes from
`traffic_core`. Its job is pure composition: wire the simulator, the server,
and the map loader together; it contains no simulation logic of its own.

**Usage:** This is the file you'd read first to understand "what actually
happens when you run the program," end to end.

### Lines 20–30: parsing arguments and loading the map

```cpp
int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "data/maps/small_map.geojson";
    const int port = (argc > 2) ? std::atoi(argv[2]) : 8080;

    TrafficSimulator& sim = TrafficSimulator::instance();
    try {
        sim.load(path);
    } catch (const MapLoadError& e) {
        std::cerr << "Map load error: " << e.what() << "\n";
        return 1;
    }
```

**Meaning:** `argc`/`argv` give the traditional C-style command-line argument
count and vector; `path` defaults to `data/maps/small_map.geojson` if no first
argument is given, `port` defaults to `8080` similarly (`std::atoi` — no error
checking if the argument isn't numeric; it'll just silently become 0).
`TrafficSimulator::instance()` is the Singleton accessor (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) returning a reference to
the one process-wide simulator. `sim.load(path)` attempts to load the GeoJSON
map, wrapped in `try`/`catch` for the specific `MapLoadError` exception type —
on failure, the error is printed to `stderr` and the process exits with status
1 (a Unix/Windows convention for "failure").

**Why:** defaulting both arguments makes the tool runnable with zero
configuration (`./traffic_simulator` just works against a bundled sample map),
while still allowing full control for anyone testing other maps or ports. This
is the first and only place `TrafficSimulator::instance()` is called in this
file for the *side effect* of construction — because it's a Singleton, every
subsequent `sim.` call anywhere else (including inside the `api` lambda below)
refers to this exact same object.

**Usage:** If the map fails to load, nothing else in `main` runs — no server
starts, no threads spawn. This is a fail-fast startup check.

### Lines 32–48: writing the static map fallback and startup diagnostics

```cpp
    // Keep the static viewer working: also dump the map to a file.
    try {
        MapLoader::write_map_json(*sim.graph(), "web_frontend/map.json");
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not write web_frontend/map.json: " << e.what() << "\n";
    }

    std::cout << "Loaded '" << path << "':\n"
              << "  nodes     = " << sim.graph()->intersections.size() << "\n"
              << "  roads     = " << sim.graph()->edges.size() << "\n"
              << "  buildings = " << sim.graph()->buildings.size() << "\n";

    // A map that parses can still be unusable: empty, or split into pieces the
    // routers cannot cross. Say so at startup rather than letting it surface
    // later as vehicles that mysteriously fail to spawn.
    const std::string warning = MapLoader::validate(*sim.graph());
    std::cout << "  validate  = " << (warning.empty() ? "OK" : warning) << "\n";
```

**Meaning:** `MapLoader::write_map_json` writes the just-loaded graph back out
as `web_frontend/map.json` — a plain static file, wrapped in its own
`try`/`catch` that only *warns* (doesn't exit) on failure, since this is a
convenience fallback, not a hard requirement. `sim.graph()` returns a pointer
to the loaded `Graph` (see [`learn/02-core-graph-model.md`](02-core-graph-model.md)),
dereferenced to print counts of intersections, edges (roads), and buildings.
`MapLoader::validate` runs a sanity check over the loaded graph and returns an
empty string if everything looks fine, or a descriptive warning otherwise.

**Why:** this is exactly the `map.json` file referenced in `api.js`'s
`loadMap()` fallback path ("2. static file next to the page") — this is the
concrete link between what `main.cpp` produces and what the frontend consumes
when the live server isn't reachable. The validate-and-print-at-startup
pattern is explicitly a design choice to surface map problems immediately, in
the terminal, rather than have them manifest later as confusing runtime
symptoms ("why won't any vehicles spawn?") that would be much harder to
diagnose without this upfront check.

**Usage:** Anyone launching `traffic_simulator` sees this startup summary
directly in the terminal — it's the first diagnostic signal that the map
loaded sanely before the server ever starts accepting connections.

### Lines 50–65: the simulation tick thread

```cpp
    // Simulation thread: advance ~20x/second in real time. The speed factor
    // (set via /api/speed) scales simulated time, but the integration step is
    // sub-stepped to stay at ~0.05 s regardless of speed -- otherwise x10 makes
    // a 0.5 s step that lets vehicles tunnel through junction zones.
    std::thread([&sim] {
        const double base = 0.05;
        while (true) {
            // tick() already scales its dt by sim.speed() internally, so only
            // the sub-step count depends on speed here -- passing base/n
            // (not base * speed / n) avoids applying speed twice, which was
            // making e.g. 5x run at 5^2 = 25x actual simulated time.
            const int n = std::max(1, static_cast<int>(std::ceil(sim.speed())));
            for (int i = 0; i < n; ++i) sim.tick(base / n);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }).detach();
```

**Meaning:** This is where the "single-threaded tick loop"
(see [`learn/07-simulation-core.md`](07-simulation-core.md)) actually lives as
an OS thread: a lambda capturing `sim` by reference, run on its own detached
`std::thread`, looping forever. Each iteration computes `n`, the number of
sub-steps this real-time frame should be split into — `sim.speed()` (the
user-adjustable playback multiplier set via `/api/speed`) is rounded up
(`std::ceil`) and floored at 1 (`std::max(1, ...)`), so e.g. speed 4.2 becomes
5 sub-steps. It then calls `sim.tick(base / n)` exactly `n` times, each with a
*small* delta-time (`0.05 / n` seconds), then sleeps 50 ms before the next
real-time frame. The comment explains the arithmetic pitfall this avoids in
detail: `tick()` internally already multiplies `dt` by `sim.speed()` — so
passing `base / n` (undoing only the sub-stepping, not the speed) is correct,
while a tempting-looking `base * speed / n` would double-apply the speed
factor, quadratically distorting playback rate (their concrete example: 5x
speed would silently become 25x).

**Why:** the sub-stepping is a numerical-integration safety measure, not a
performance one — the comment states the concrete failure mode directly: at
high speed multipliers, a single large `dt` step could let a fast vehicle's
position update skip clean over (tunnel through) a junction's collision/gap-check
zone in one step, since the simulator only checks state at discrete tick
boundaries. Splitting the same total simulated time into more, smaller steps
keeps each step's position delta small enough that junction zones aren't
skipped. This is squarely in the territory of the "junction overlap" known
limitation from [`learn/07-simulation-core.md`](07-simulation-core.md) — it's
mitigated here by sub-stepping, but that's damage control around a
tick-based model, not a structural fix; a large enough speed value still
raises `n` correspondingly, so the mitigation scales with the very thing
that's causing the risk rather than eliminating it outright.

**Usage:** This detached thread is the *only* place `sim.tick()` is ever
called in this file — it's the heartbeat of the whole simulation, running
completely independently of the HTTP accept loop covered above. This is the
concrete answer to "how is networking coordinated with the tick loop": they
are two separate OS threads (this one, plus one detached thread per HTTP
connection from `HttpServer::run()`), and neither blocks on the other directly
— coordination between them happens entirely through `TrafficSimulator`'s
internal locking (`mtx_`, `snapshot_mtx_`, `bin_snapshot_mtx_`, per
[`learn/07-simulation-core.md`](07-simulation-core.md)), not through anything
in this file.

### Lines 67–71: starting the publisher

```cpp
    // Publisher: rebuilds the /api/state payload 10x a second (see
    // publish_loop()) so serving it costs a pointer copy instead of a lock on
    // the simulation. Vehicle positions don't wait on this -- tick() publishes
    // its own binary snapshot every tick (20 Hz), served from /api/stateBin.
    sim.start_publishing();
```

**Meaning:** One call, starting a *third* background thread inside
`TrafficSimulator` itself (`publish_loop`, not defined in this file — see
`cpp_backend/src/simulation/TrafficSimulator.cpp`). This is the concrete
mechanism the comment refers to: `publish_loop()` runs on its own thread,
rebuilding a full JSON `state_json()` string every 100 ms and storing it behind
a *separate* mutex (`snapshot_mtx_`, distinct from the simulation's own `mtx_`)
as a `std::shared_ptr<const std::string>`. `TrafficSimulator::state_snapshot()`
then just copies that shared pointer under a very short-lived lock on
`snapshot_mtx_` — never touching `mtx_`, the mutex the tick thread holds while
actually mutating simulation state.

**Why:** this is the answer to the "how is the accept-loop/tick-loop
interaction coordinated" question in concrete terms — it isn't a naive "HTTP
handler thread locks the same mutex the tick loop uses and blocks until the
tick finishes." That design would let a slow HTTP client (or a burst of many
concurrent requests, easily produced given the "one thread per connection"
model) stall the simulation thread, or force `/api/state` requests to queue up
behind however long the current tick takes. Instead, `/api/state` reads a
*cached, pre-serialized* snapshot behind its own lightweight mutex — the
comment in `TrafficSimulator.hpp` is explicit that `snapshot_mtx_` is
deliberately its own mutex "reusing `mtx_` would defeat the entire point:
readers [would] block on writers." So there are effectively three independent
timing domains running concurrently: the tick thread (mutating state,
~20 Hz), the publisher thread (serializing a JSON copy, 10 Hz), and however
many HTTP handler threads exist at a given moment (each just copying a
`shared_ptr`, no meaningful lock contention). `/api/stateBin` takes this even
further per its own comment: `tick()` itself publishes a *binary* snapshot on
every tick (20 Hz) behind yet another dedicated mutex
(`bin_snapshot_mtx_`), so vehicle positions specifically don't even wait on
the publisher thread's slower 10 Hz cadence — only the JSON convenience
payload does.

**Usage:** Every `/api/state` and `/api/stateBin` request in the `api` lambda
below reads from these published snapshots rather than computing anything live
against the running simulation, which is exactly why a burst of browser
polling can never stall a tick.

### Lines 73–91: the JSON string escaper

```cpp
    // Minimal JSON string escaper for the few places main() builds a reply
    // containing text that came from outside (an import warning, a parser
    // message). Everything else here emits fixed literals.
    auto jstr_ = [](const std::string& in) {
        std::string out = "\"";
        for (char c : in) {
            if (c == '"' || c == '\\') {
                out += '\\';
                out += c;
            } else if (c == '\n') {
                out += "\\n";
            } else if (static_cast<unsigned char>(c) < 0x20) {
                continue;   // control characters would break the document
            } else {
                out += c;
            }
        }
        return out + "\"";
    };
```

**Meaning:** A local lambda, `jstr_`, that turns an arbitrary `std::string`
into a valid JSON string literal (including the surrounding quotes): it
escapes `"` and `\` by prefixing a backslash, converts newlines to the
two-character escape `\n`, silently drops other control characters
(byte values below `0x20`, i.e. below the space character) since they'd
produce invalid JSON if emitted raw, and passes everything else through
unchanged.

**Why:** the comment is explicit that this exists only because a handful of
endpoints (like `/api/importMap`, seen below) need to embed *externally
sourced* text — an error message from a parser, a validation warning — inside
a JSON reply, and that text might itself contain quote characters or other
JSON-breaking bytes. Every other JSON literal built in this file is a fixed,
hand-written string the author controls, so it doesn't need escaping — this
lambda is scoped narrowly to the actual risk.

**Usage:** Called from `/api/importMap`'s error and warning paths, below.

### Lines 93–99: the `api` lambda signature and its `get` helper

```cpp
    // REST API dispatch.
    auto api = [&sim, &jstr_](const std::string& p, const HttpServer::Params& params,
                              const std::string& body) -> std::string {
        auto get = [&](const std::string& key, const std::string& def = "") {
            auto it = params.find(key);
            return it == params.end() ? def : it->second;
        };
```

**Meaning:** This is the `ApiHandler` — the exact callable `HttpServer.hpp`'s
`ApiHandler` alias describes: `(path, params, body) -> std::string`. It
captures `sim` and `jstr_` by reference (`[&sim, &jstr_]`), meaning every
branch inside can call methods on the one Singleton simulator and use the
escaper defined above. `get` is a small local convenience: look up a query
param by key, returning a default (empty string, unless overridden) if it's
absent — avoiding repeating `params.find(key) == params.end() ? ... : ...`
throughout the ~200 lines of dispatch that follow.

**Why:** this lambda is the concrete realization of the inversion-of-control
point made in the `WebSocketServer.hpp` walkthrough — `HttpServer` never
includes `TrafficSimulator.hpp` and has no idea what a `TrafficSimulator` is;
`main.cpp` is the only place that closes that gap, by handing a
`TrafficSimulator`-aware function object into a generically-typed constructor
parameter.

**Usage:** Passed as the third constructor argument to `HttpServer` at the
bottom of `main`. Every branch below runs on whatever HTTP handler thread
called it — i.e., concurrently with every other in-flight request and with the
tick thread, coordinated only through whatever locking `TrafficSimulator`'s own
methods do internally (see [`learn/07-simulation-core.md`](07-simulation-core.md)
for the specifics of `mtx_`).

### Lines 100–139: read endpoints and `/api/spawn` / `/api/event`

```cpp
        if (p == "/api/map") return sim.map_json();
        if (p == "/api/history/range") return sim.history_range_json();
        if (p == "/api/history/at") {
            const std::string agos = get("ago");
            double ago = 0.0;
            if (!agos.empty()) {
                try { ago = std::stod(agos); } catch (const std::exception&) { ago = 0.0; }
            }
            return sim.history_state_at_json(ago);
        }
        if (p == "/api/state") {
            // Lock-free: served from the published snapshot, so a heavy tick
            // can never stall the browser. The direct call is only the cold
            // start, before the publisher has produced anything.
            if (auto snap = sim.state_snapshot()) return *snap;
            return sim.state_json();
        }
        if (p == "/api/spawn") {
            const std::string id = sim.spawn_vehicle(get("from"), get("to"), get("type", "car"));
            return "{\"id\":\"" + id + "\"}";
        }
        if (p == "/api/event") {
            const std::string durs = get("dur");
            const double dur = durs.empty() ? 0.0 : std::stod(durs);
            const std::string id = sim.trigger_event(get("type"), get("target"), dur);
            return "{\"id\":\"" + id + "\"}";
        }
```

**Meaning:** A long, flat chain of `if (p == "/api/...") { ...; return ...; }`
statements — this is the entire routing table, no framework, no table lookup,
just sequential string comparison against the path. `/api/map` returns the
graph as JSON directly from `sim`. `/api/history/range` and `/api/history/at`
expose the playback scrubber's backing store (`HistoryStore`, per
[`learn/00-syllabus.md`](00-syllabus.md)'s module 8 scope) — `/api/history/at`
parses an `ago` query param as a double, defaulting to 0 on a missing or
unparseable value (note the `try`/`catch` here specifically, versus `/api/event`'s
`dur` a few lines down, which has no such guard — an inconsistency worth
noticing). `/api/state` is the snapshot-read path just discussed: it prefers
`sim.state_snapshot()` (the publisher's cached copy) and only falls back to a
live, computed `sim.state_json()` if no snapshot exists yet — i.e., only in the
narrow startup window before `publish_loop()`'s first 100 ms tick has landed.
`/api/spawn` and `/api/event` both call a `sim.` mutator and wrap the resulting
ID in a small hand-written JSON object.

**Why:** the flat `if`-chain (rather than a `std::map<std::string, Handler>`
lookup table) is a readability-over-elegance tradeoff — with ~30 endpoints, a
lookup table would technically be faster and shorter, but the linear chain
keeps each endpoint's logic inline and grep-able, which matters more for a
project this size than the O(1) vs. O(n) dispatch cost (n is small, and this
runs once per HTTP request, not in a hot loop). The inconsistent exception
handling around `std::stod` (`/api/history/at` catches, `/api/event`'s `dur`
does not) is a real inconsistency — the latter would throw uncaught if `dur`
is present but not a valid number, which propagates up into
`HttpServer::handle_client`'s `try`/`catch` around `api_handler_(...)` and gets
turned into a `400 Bad Request` rather than crashing the thread — so it's not
a stability bug, just a slightly less specific error message than the guarded
path would give.

**Usage:** `/api/spawn` and `/api/event` are exactly what
`web_frontend/js/cpp_api.js`... — actually, note that `cpp_api.js` in this
repo is an **empty file** (0 bytes); the frontend calls these endpoints through
`web_frontend/js/api.js`'s `Sim.spawn` and `Sim.event` wrappers instead (see
"How the frontend talks to this," below).

### Lines 140–192: playback and tuning controls

```cpp
        // Two separate switches that were wired to the same one. Auto-spawn
        // keeps the city populated; random events are what happens TO it.
        // Turning off traffic generation should not also stop accidents.
        auto flag = [&](const std::string& key) {
            const std::string v = get(key);
            return v == "1" || v == "true";
        };
        if (p == "/api/auto") {
            sim.set_auto_spawn(flag("on"));
            return "{\"ok\":true}";
        }
        if (p == "/api/randomEvents") {
            sim.set_random_events(params.count("enabled") ? flag("enabled")
                                                          : flag("on"));
            return "{\"ok\":true}";
        }
        ...
        if (p == "/api/junctionApproach") {
            sim.set_junction_approach(num("value", 14.0));
            return "{\"ok\":true,\"value\":" +
                   std::to_string(sim.junction_approach()) + "}";
        }
```

**Meaning:** `flag` is a second small local helper, parsing a query param as a
boolean (`"1"` or `"true"`, everything else is falsy). `/api/pause`,
`/api/resume`, `/api/speed`, `/api/auto`, and `/api/randomEvents` are all thin
wrappers calling one `sim.set_*` mutator each. The comment above `/api/auto`
and `/api/randomEvents` documents a real, specific bug-fix-shaped design
decision: these were apparently once controlled by a single shared switch, and
someone deliberately split them because turning off auto-spawn (new vehicles
entering the city) shouldn't also silence accident/random-event generation —
they're conceptually independent ("keeps the city populated" vs. "what happens
TO it"). `/api/randomEvents` accepts *either* an `enabled` param or an `on`
param, with a comment explaining why: `enabled` matches some external
reference API's naming, `on` matches this codebase's own convention elsewhere
— so the endpoint accepts both rather than forcing every caller to remember
which name applies here. `/api/junctionApproach` (further down, alongside
`/api/target`, `/api/spawnRate`) tunes live simulation parameters — the
comment for `junctionApproach` specifically identifies it as the distance (in
meters) at which a driver starts deciding whether to give way at a junction,
directly relevant to the junction-overlap behavior discussed in
[`learn/07-simulation-core.md`](07-simulation-core.md); it's exposed live (not
just a startup constant) "so it can be dialled in against moving traffic."

**Why:** exposing tuning knobs like `junctionApproach`, `target` (vehicle
count), and `spawnRate` as live HTTP endpoints rather than compile-time
constants or config-file values is a debugging/tuning convenience — you can
watch the simulation and adjust these values interactively from the browser
UI's controls, immediately seeing the effect, rather than restarting the
process for every experiment.

**Usage:** These map directly to controls in `web_frontend/js/controls.js`
(not read in full for this module, but referenced by name in `api.js`'s `Sim`
object, e.g. `Sim.setJunctionApproach`).

### Lines 194–221: vehicle operations and the binary state endpoint

```cpp
        if (p == "/api/vehicle/detail") return sim.vehicle_detail_json(get("id"));
        if (p == "/api/removeVehicle") {
            return sim.remove_vehicle(get("id")) ? ok_true
                 : "{\"ok\":false,\"error\":\"no such vehicle\"}";
        }
        if (p == "/api/clearVehicles") {
            return "{\"ok\":true,\"removed\":" + std::to_string(sim.clear_vehicles()) + "}";
        }
        if (p == "/api/stateBin") {
            // Lock-free, same as /api/state: served from the copy tick()
            // publishes every 50ms, so a request never blocks behind a tick.
            // The direct call is only the cold start, before the first tick.
            if (auto snap = sim.binary_snapshot()) {
                return std::string(reinterpret_cast<const char*>(snap->data()), snap->size());
            }
            std::vector<uint8_t> bin_buf;
            sim.encode_binary_snapshot(bin_buf);
            return std::string(reinterpret_cast<const char*>(bin_buf.data()), bin_buf.size());
        }
        if (p == "/api/vehicle") {
            const std::string id = get("id");
            bool done = false;
            if (!get("algorithm").empty()) done |= sim.set_vehicle_algorithm(id, get("algorithm"));
            if (!get("to").empty())        done |= sim.retarget_vehicle(id, get("to"));
            return done ? ok_true : "{\"ok\":false,\"error\":\"vehicle update failed\"}";
        }
```

**Meaning:** Most of these are direct, boolean-gated wrappers around `sim.`
mutators, returning a small hand-written success/error JSON object. `/api/stateBin`
is notable: unlike every other endpoint, its return value is *not* JSON text —
it's raw binary bytes (a `VehicleSnapshotBin`-shaped buffer per
`TrafficSimulator.hpp`'s comments) wrapped in a `std::string` purely as a byte
container (`reinterpret_cast<const char*>` reinterprets the `uint8_t` buffer as
`char` data, with no encoding transformation happening). It follows the same
snapshot-preferred, live-fallback pattern as `/api/state`, but reading from
`binary_snapshot()` (refreshed on *every tick*, 20 Hz, per the earlier
walkthrough of `TrafficSimulator::tick()`'s locking) rather than
`state_snapshot()` (refreshed by the separate publisher thread at 10 Hz) —
this is the higher-frequency, lower-overhead channel specifically for vehicle
positions.

**Why:** shipping vehicle positions as a compact binary format instead of JSON
is a bandwidth/parsing-cost optimization — with potentially hundreds of
vehicles updating 20 times a second, JSON's per-field key names and text-number
encoding is meaningfully heavier than a fixed binary layout the frontend can
parse with a typed array view. `/api/vehicle`'s `done |=` pattern (accumulating
across two independent, optional sub-operations — changing algorithm and/or
retargeting) is a compact way to let a single call do either, both, or (if
both params are empty) neither, in which case it correctly reports failure.

**Usage:** `/api/stateBin` is fetched by `Sim`'s consumer via
`web_frontend/js/api.js`'s `loadBinaryState()` (seen above), which calls
`res.arrayBuffer()` — direct evidence the frontend expects raw bytes here, not
JSON, corroborating the binary-format read above.

### Lines 223–316: live map editing and map file management

```cpp
        if (p == "/api/addRoad") {
            auto endpoint = [&](const std::string& prefix) -> std::pair<std::string, bool> {
                const std::string kind = get(prefix + "Kind");
                if (kind == "road" || kind == "free") {
                    const double x = num(prefix + "X", 0.0), y = num(prefix + "Y", 0.0);
                    return kind == "road" ? std::make_pair(sim.split_road_near(x, y), false)
                                          : std::make_pair(sim.add_junction_at(x, y), true);
                }
                return {get(prefix), false};
            };
            ...
        }
        ...
        if (p == "/api/importMap") {
            // The browser fetched the area from OpenStreetMap, converted it to
            // GeoJSON and POSTed it here. The fetch happens there rather than
            // in this process on purpose: Overpass is HTTPS, and the browser
            // already has TLS -- adding an HTTPS client to a project with no
            // dependencies would be a large price for one feature.
            const std::string name = get("name", "osm-import");
            const auto r = sim.import_map(name, body);
            if (!r.ok) {
                return std::string("{\"ok\":false,\"error\":") + jstr_(r.message) + "}";
            }
            if (r.message.empty()) return ok_true;
            return std::string("{\"ok\":true,\"warning\":") + jstr_(r.message) + "}";
        }
```

**Meaning:** `/api/addRoad` is the most structurally complex endpoint here: a
local `endpoint` lambda resolves either end of a new road from three possible
shapes — an existing junction ID, a point on an existing road (which gets
split there via `sim.split_road_near`), or empty ground (creates a new
junction via `sim.add_junction_at`) — returning both the resolved ID and
whether that endpoint was newly created ("free"). The comment explains a
subtlety: resolving both endpoints by coordinates server-side (rather than
requiring the browser to pre-compute IDs) means "both ends on the same road"
naturally works correctly, because the second `endpoint(...)` call's
`split_road_near` sees the graph *after* the first split already happened.
Further down, a one-way flag gets force-overridden to two-way if either
endpoint was freshly created, since a brand-new dead-end junction needs to
stay reachable in both directions. `/api/importMap` receives a POST body (this
is the concrete case the `ApiHandler`'s `body` parameter and the whole
body-reading logic in `WebSocketServer.cpp` exists for) containing GeoJSON
that the *browser* already fetched from OpenStreetMap's Overpass API — the
comment explicitly justifies this division of labor: Overpass requires HTTPS,
and adding a TLS-capable HTTP client to this "no dependencies" C++ project
just for one feature isn't worth it when the browser already has a perfectly
good HTTPS client built in.

**Why:** the `/api/importMap` comment is a clean, explicit statement of an
architectural boundary: this server's own outbound networking capability is
effectively zero (it only ever accepts connections, never initiates HTTPS
requests to third parties) — the browser does the fetching, this process only
ever *ingests* what's POSTed to it. This reinforces the "small, dependency-free"
theme established from the very first line of `WebSocketServer.hpp`.

**Usage:** These map-editing endpoints correspond to `web_frontend/js/api.js`'s
`Sim.addRoad`, `Sim.removeRoad`, `Sim.setLight`, `Sim.addBuilding`, etc. — the
in-browser map editor UI drives these directly.

### Lines 319–322: constructing and running the server

```cpp
    HttpServer server(port, "web_frontend", api);
    server.run();  // blocking accept loop
    return 0;
}
```

**Meaning:** The `HttpServer` constructed here takes `port` (parsed from
`argv[2]` or defaulting to 8080), the literal string `"web_frontend"` as the
document root (a relative path — this only resolves correctly if the process
is launched from the repository root, or wherever `web_frontend/` sits
relative to the current working directory), and `api`, the dispatch lambda
built above. `server.run()` is called and, per its own comment here and the
`.cpp` walkthrough above, blocks forever inside its accept loop — this is the
last thing `main()` ever does under normal operation. `return 0;` is
technically unreachable unless `run()` itself returns early (e.g. `socket()`,
`bind()`, or `listen()` failing) — in which case the process still exits
successfully (status 0), even though the server never actually served
anything, which is a rough edge worth naming: a `bind()` failure prints a
diagnostic to `stderr` but the process's own exit code doesn't reflect that
failure the way the earlier `sim.load()` failure path does (`return 1`).

**Why:** this is the complete startup sequence, now traceable end to end: load
map -> write static fallback -> print diagnostics -> validate -> spawn tick
thread -> start publisher thread -> build API dispatch lambda -> construct
`HttpServer` -> block in `run()`. Every one of the concurrently running pieces
(tick thread, publisher thread, and however many per-connection threads exist
at any instant) was started from this one function, and none of them are ever
joined — the process's lifetime is entirely governed by `server.run()`'s
blocking accept loop, ended only by the OS killing the process (e.g. Ctrl+C).

**Usage:** This is the object whose `run()` call is the entry point into
everything covered in the `WebSocketServer.cpp` section above.

**Check yourself:**
- Trace every background thread `main()` starts before it calls `server.run()`. What does each one do, and which mutex (if any) does it use to publish its results safely to the HTTP handler threads?
- Why does `/api/importMap` receive a POST body instead of query parameters, and why does the *browser*, not this C++ process, fetch from OpenStreetMap?
- `/api/event`'s `dur` parsing has no `try`/`catch` around `std::stod`, while `/api/history/at`'s `ago` parsing does. Trace what actually happens if a client sends `/api/event?dur=notanumber` — does the process crash, and if not, why not?
- If `bind()` fails inside `HttpServer::run()`, what does the process's final exit code end up being, and is that consistent with how `sim.load()` failures are handled a few lines earlier in `main()`?

---

## How the frontend talks to this

`web_frontend/js/api.js` is the browser-side counterpart to everything above,
and it confirms the protocol directly: every call is a plain `fetch(...)` —
`fetch("/api/map", ...)`, `fetch("/api/state", ...)`, `fetch("/api/stateBin", ...)`,
`fetch(qs ? "${path}?${qs}" : path, ...)` inside the shared `control()` helper
used by the `Sim` object's ~25 methods. There is no `new WebSocket(...)`
anywhere in this module. The frontend is a REST/AJAX client polling a plain
HTTP server, exactly matching what `WebSocketServer.cpp` actually implements —
the class name is the only place "WebSocket" appears in this codebase's
networking story.

Two details in `api.js` are worth connecting back to the C++ side directly:

- `loadMap()` tries `/api/map` first and falls back to the static
  `../map.json` file if that fetch fails — this is precisely the fallback
  `main.cpp` sets up by calling `MapLoader::write_map_json(..., "web_frontend/map.json")`
  right after loading the map, so the same page keeps working (in a read-only,
  non-live capacity) even if it's opened via a plain static file server instead
  of `traffic_simulator` itself.
- `loadBinaryState()` calls `res.arrayBuffer()` instead of `res.json()` —
  direct confirmation that `/api/stateBin` is genuinely binary, matching the
  `reinterpret_cast<const char*>` byte-copying seen in `main.cpp`'s handler for
  that endpoint.

`web_frontend/js/cpp_api.js`, despite its name suggesting it might be the
frontend's C++ bridge, is an empty file (0 bytes) in this repository —
`api.js` is the module that actually fulfills that role. If you go looking for
the "real" API client, `api.js` is it.

There is no coordination protocol beyond ordinary polling: the frontend
doesn't hold a persistent connection, doesn't receive push notifications, and
doesn't know anything about the tick thread, the publisher thread, or the
locking inside `TrafficSimulator`. It just issues plain HTTP requests on
whatever cadence its own render loop dictates, and the server-side snapshot
publishing scheme described above is precisely what makes that safe to do at
a high polling rate without ever stalling behind a simulation tick.

If [`learn/06-events-observer.md`](06-events-observer.md) covers how
`TrafficController` broadcasts events to observers internally, note that none
of that Observer-pattern machinery reaches the browser directly — events
surface to the frontend only indirectly, as data embedded in the periodically
polled `/api/state` JSON payload (an `events[]` array, per the shape comment
in `api.js`'s `loadState()`), not as any kind of pushed notification. The
Observer subject and the HTTP layer are two unrelated mechanisms that happen
to feed the same eventual JSON blob.
