# Module 01 — Orientation

**File covered:** [`CMakeLists.txt`](../CMakeLists.txt) (repo root, 124 lines)

This is the build graph for the whole project. Before touching any C++ source,
you need to know what actually gets compiled into what — that's what this file
answers. Read it once and you'll know exactly which files matter for the
library, which for the app, and which are test-only.

---

### Lines 1–2: minimum version and project declaration

```cmake
cmake_minimum_required(VERSION 3.16)
project(TrafficSimulator CXX)
```

**Meaning:** `cmake_minimum_required` tells CMake "refuse to configure this
project on any CMake older than 3.16" — it also implicitly sets CMake's
*policy* baseline to that version, so newer CMake versions don't silently
change behavior out from under this file. `project(TrafficSimulator CXX)`
declares the project's name (`TrafficSimulator`, used in IDE generators and
default install paths) and its language, `CXX` — i.e. C++ only, no C compiler
needed.

**Why:** CMake 3.16 (Nov 2019) is the version where several conveniences this
file relies on elsewhere became reliable across platforms — it's an
unremarkable, safe floor rather than a version chosen for one specific
feature. Declaring `CXX` explicitly (instead of letting CMake guess from file
extensions) avoids CMake probing for a C compiler that this project doesn't
need.

**Usage:** Every `add_library`/`add_executable` call later in the file
depends on `project()` having run first — it's what initializes the compiler
toolchain variables (`CMAKE_CXX_COMPILER`, etc.) that the rest of the file
uses implicitly.

**Check yourself:**
- What would happen if `project()` were called before `cmake_minimum_required()`? (CMake actually errors on this — order matters.)
- Why declare `CXX` instead of leaving the language list empty?

---

### Lines 4–24: defaulting the build type to Release

```cmake
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type: Debug, Release, RelWithDebInfo or MinSizeRel" FORCE)
    message(STATUS "No build type given, defaulting to Release. "
                   "Use -DCMAKE_BUILD_TYPE=Debug for an unoptimised build.")
endif()
```

**Meaning:** `CMAKE_BUILD_TYPE` is the variable single-config generators
(Makefiles, Ninja) use to decide which optimization flags to pass
(`Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel`). `CMAKE_CONFIGURATION_TYPES`
is the equivalent concept for *multi*-config generators (Visual Studio,
Xcode), which pick the configuration at build time instead of configure time
— so it's only ever set when using one of those generators. The `if` guard
fires only when *neither* is set, i.e. only for single-config generators
where the user didn't pass `-DCMAKE_BUILD_TYPE=...`. Inside, `set(...
CACHE STRING ... FORCE)` writes `CMAKE_BUILD_TYPE=Release` into the CMake
cache, overwriting anything already cached there. `message(STATUS ...)`
prints an informational line during configuration so the choice is visible,
not silent.

**Why:** the comment directly above this block (lines 4–18 in the file)
explains the motivation better than paraphrase would: without this default,
`cmake -S . -B build -G Ninja` — "the command everyone actually types" —
leaves `CMAKE_BUILD_TYPE` empty, which means *no* optimization flags at all,
not even `-O0` explicitly chosen. The binary still runs, every test still
passes, so nothing *looks* wrong — it's just several times slower, silently.
The comment cites a concrete consequence: a benchmark in this project was
once reported as "13x faster" purely because the *baseline* run happened to
be built with no optimization flags at all; the honest number, once both
sides were built the same way, was 3.7x. This block exists specifically to
make that class of mistake structurally impossible for anyone who doesn't
pass `-DCMAKE_BUILD_TYPE` explicitly.

**Usage:** This is a project-wide policy, not tied to any one target — it
affects every `add_library`/`add_executable` below by determining what
optimization flags the compiler driver adds. If you're ever benchmarking two
versions of this code against each other, this is the line that guarantees
both were compiled the same way *by default* — but if you explicitly pass
`-DCMAKE_BUILD_TYPE=Debug`, this block does nothing (the `if` doesn't fire),
so it's still possible to accidentally benchmark a debug build if you set the
flag yourself and forget about it.

**Check yourself:**
- Why does the guard check `CMAKE_CONFIGURATION_TYPES` at all, instead of just checking `CMAKE_BUILD_TYPE`?
- If you ran `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug`, would this block override you back to Release? Why or why not?

---

### Lines 26–27: the C++ standard

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

**Meaning:** `CMAKE_CXX_STANDARD 20` requests C++20 for every target
configured after this point (unless a target overrides it). `CMAKE_CXX_STANDARD_REQUIRED ON`
makes that request a hard requirement — if the detected compiler cannot
provide C++20, configuration *fails* instead of silently falling back to an
older standard.

**Why:** without `_REQUIRED ON`, `CMAKE_CXX_STANDARD` is only a *preference*
— CMake will quietly compile against the newest standard the compiler
actually supports if C++20 isn't available, which could mean code using
C++20 features (e.g. concepts, ranges, `<format>`-adjacent features) fails
with confusing syntax errors far from the real cause instead of a clear
"your compiler doesn't support C++20" message at configure time.

**Usage:** Every target added later (`traffic_core`, `traffic_simulator`,
all the test executables) inherits this. This is the project-wide
confirmation of the "C++20 backend" claim in the top-level project
description.

**Check yourself:**
- What's the practical difference between omitting `CMAKE_CXX_STANDARD_REQUIRED` entirely versus setting it `ON`?

---

### Lines 29–33: compiler warning flags

```cmake
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra)
endif()
```

**Meaning:** `MSVC` is a CMake variable that's true when the active compiler
is Microsoft's `cl.exe` (as opposed to GCC/Clang). `add_compile_options`
appends flags to every compilation in every target from this point in the
file onward. `/W3` is MSVC's "level 3" warning set (a reasonable default,
one below the strictest `/W4`). `-Wall -Wextra` is the GCC/Clang idiom for
"most warnings" — despite the name, `-Wall` does *not* mean "all warnings."

**Why:** MSVC and GCC/Clang use completely different command-line flag
syntax, so the project needs to branch on which compiler is active to ask
for "a sensible amount of warnings" on either. This is a portability
necessity, not a stylistic choice — `-Wall` passed to `cl.exe` isn't
meaningful, and `/W3` isn't meaningful to GCC.

**Usage:** Because this uses `add_compile_options` (not
`target_compile_options`), it's global to *everything* configured after
this line — the library, the executable, and every test target. It's a
blunt, project-wide instrument.

**Check yourself:**
- Why might a project choose `-Wall -Wextra` over the stricter `-Wall -Wextra -Werror` (which turns warnings into errors)? What's the tradeoff?

---

### Lines 35–59: the `BACKEND_SOURCES` list

```cmake
set(BACKEND_SOURCES
    cpp_backend/src/core/Graph.cpp
    cpp_backend/src/core/Intersection.cpp
    ...
    cpp_backend/src/simulation/TrafficSimulator.cpp
)
```

**Meaning:** `set(BACKEND_SOURCES ... )` defines a CMake list variable
containing every `.cpp` file that will be compiled into the shared backend
library. It's a flat list of 20 paths, one per source file, spanning
`core/`, `routing/`, `utils/`, `server/`, `events/`, `vehicles/`, and
`simulation/`.

**Why:** the comment directly above it ("Implemented source files (empty
stubs are omitted so the build does not fail at link time)") is important:
this is *not* simply every `.cpp` under `cpp_backend/src/`. It's a curated
list — if a header exists without a corresponding `.cpp` being implemented
yet, that `.cpp` is deliberately left out of this list rather than compiled
as an empty stub, because an empty translation unit that's supposed to
define symbols the linker expects would produce link errors, not compile
errors — a much more confusing failure mode to hit by accident. Keeping this
list explicit (rather than a glob like `file(GLOB ...)`) is also a common
CMake best practice, because `file(GLOB)` doesn't automatically pick up new
files without a re-run of CMake, so an explicit list makes the build's
dependency set unambiguous and reproducible.

**Usage:** This variable is consumed exactly once, in the very next
statement (`add_library(traffic_core STATIC ${BACKEND_SOURCES})`). Notice
`Zone.hpp` (mentioned in the architecture overview) has **no** corresponding
`.cpp` in this list — a strong signal that `Zone` is currently a
header-only type (see `learn/02-core-graph-model.md` for what that implies
about its contents, e.g. no out-of-line member function definitions).

**Check yourself:**
- If you added a new file `cpp_backend/src/core/Lane.cpp` but forgot to add it to `BACKEND_SOURCES`, what would happen when you built the project? Would CMake warn you?
- Why list source files explicitly instead of using `file(GLOB_RECURSE ...)`?

---

### Lines 61: the threading dependency

```cmake
find_package(Threads REQUIRED)
```

**Meaning:** Asks CMake to locate the platform's threading library (POSIX
threads on Linux/macOS, the Windows threading runtime on Windows) and expose
it as the imported target `Threads::Threads`. `REQUIRED` means configuration
fails outright if no threading support is found, rather than continuing with
threading silently unavailable.

**Why:** the project claims a "single-threaded tick loop" as an architectural
decision (see `learn/07-simulation-core.md`), but that doesn't mean *no*
threads exist anywhere in the process — the fact that `find_package(Threads REQUIRED)`
is here at all tells you something in this codebase (most likely the
WebSocket/HTTP server accepting connections concurrently with the simulation
loop, see `learn/09-server-and-entrypoint.md`) needs real OS thread support,
even if the simulation state itself is only ever mutated from one thread.

**Usage:** The result, `Threads::Threads`, is linked into `traffic_core`
two lines later. Because it's a `PUBLIC` link (see below), anything that
links `traffic_core` also automatically gets the threading library linked
in — you never need to call `find_package(Threads)` again in this file.

**Check yourself:**
- Given the "single-threaded tick loop" limitation, what's a plausible reason this codebase still needs `Threads::Threads`?

---

### Lines 63–68: the `traffic_core` static library target

```cmake
add_library(traffic_core STATIC ${BACKEND_SOURCES})
target_include_directories(traffic_core PUBLIC cpp_backend/include)
target_link_libraries(traffic_core PUBLIC Threads::Threads)
if(WIN32)
    target_link_libraries(traffic_core PUBLIC ws2_32)  # Winsock for the HTTP server
endif()
```

**Meaning:** `add_library(traffic_core STATIC ${BACKEND_SOURCES})` declares
a static-library build target named `traffic_core`, compiled from every file
in `BACKEND_SOURCES`. `STATIC` means the result is a `.lib`/`.a` archive that
gets baked directly into whatever links it (as opposed to `SHARED`, a `.dll`/`.so`
loaded at runtime). `target_include_directories(... PUBLIC cpp_backend/include)`
tells the compiler where to find this library's headers (`#include "core/Graph.hpp"`
resolves relative to `cpp_backend/include`), and `PUBLIC` means that
include path is *also* exposed to anything that links `traffic_core` — so
`traffic_simulator` and every test executable automatically get
`cpp_backend/include` on their include path too, without repeating it.
`target_link_libraries(traffic_core PUBLIC Threads::Threads)` links the
threading library in, again `PUBLIC` so it propagates transitively. The
`if(WIN32)` block conditionally links `ws2_32` — the Windows Sockets 2
import library — only when building on Windows, with an inline comment
explaining why: "Winsock for the HTTP server."

**Why:** `STATIC` (rather than `SHARED`) is the simpler choice for a project
this size — no runtime `.dll` search-path headaches, no ABI-versioning
concerns, at the cost of every consumer relinking when the library changes
(a non-issue for a project built from source each time). Using `PUBLIC` for
both the include directory and the Threads link is what makes downstream
targets (`traffic_simulator`, the test executables) work with zero extra
CMake code — this is CMake's "usage requirements" system doing real work:
declare a dependency once, on the library, and every consumer inherits it
automatically. `ws2_32` is only linked on Windows because Berkeley sockets
(used natively on Linux/macOS) require an explicit initialization/link step
on Windows that other platforms don't need — this is the standard way C++
networking code stays portable across Windows and POSIX systems.

**Usage:** This is the target every other executable in the file links
against. If you're tracing "where does `Graph::addRoad` actually get
compiled," the answer is: as part of `traffic_core`, because `Graph.cpp` is
in `BACKEND_SOURCES`, which was passed to this `add_library` call.

**Check yourself:**
- If `target_include_directories` used `PRIVATE` instead of `PUBLIC`, what would break when compiling `traffic_simulator`?
- Why is `ws2_32` guarded by `if(WIN32)` instead of always being linked?

---

### Lines 70–71: the `traffic_simulator` executable

```cmake
add_executable(traffic_simulator cpp_backend/src/main.cpp)
target_link_libraries(traffic_simulator PRIVATE traffic_core)
```

**Meaning:** Declares the actual shippable program, compiled from exactly
one file, `main.cpp` — everything else it needs comes from linking
`traffic_core`. `PRIVATE` here means "`traffic_simulator` uses
`traffic_core`, but nothing links against `traffic_simulator` itself, so
there's no transitive propagation to worry about" — `PRIVATE` is
appropriate for an executable (nothing ever links *against* an executable)
where `PUBLIC` was appropriate for the library.

**Why:** keeping `main.cpp` as the *only* file directly in the executable,
with everything else pulled in through the library, is what makes the test
executables further down possible — they can link `traffic_core` and get
the entire simulation engine without also pulling in `main`'s entry-point
logic (which would otherwise conflict with the test framework's own `main`).

**Usage:** This is the target you'd run to actually launch the simulator
and its server. See `learn/09-server-and-entrypoint.md` for what `main.cpp`
does once built.

**Check yourself:**
- Why does `traffic_simulator` link `traffic_core` as `PRIVATE` while `traffic_core` links `Threads::Threads` as `PUBLIC`? What governs the choice?

---

### Lines 73–123: the test and tooling executables

```cmake
add_executable(test_pathfinder_unit  tests/unit_tests/test_pathfinder.cpp)
target_link_libraries(test_pathfinder_unit PRIVATE traffic_core)
...
add_executable(route_bench tools/route_bench.cpp)
target_link_libraries(route_bench PRIVATE traffic_core)
...
add_executable(test_lane_changes_unit tests/unit_tests/test_lane_changes.cpp)
target_link_libraries(test_lane_changes_unit PRIVATE traffic_core)
```

**Meaning:** A repeated pattern — one `add_executable` + one
`target_link_libraries(... PRIVATE traffic_core)` pair per test file. Each
pair is a fully standalone executable with its own `main()` (typically
inside the test file itself), linking the shared engine code from
`traffic_core`. The naming convention encodes both the subsystem under test
and the *kind* of test: `_unit` (unit tests), `_edge` (edge-case tests, under
`tests/edge_cases/`), and `_stress` (stress tests, under `tests/stress_tests/`).
One target, `route_bench` (under `tools/`, not `tests/`), isn't a test at all
— it's a benchmarking tool, consistent with the "13x vs 3.7x" story in the
Release-default comment near the top of the file: this project has clearly
been burned by unreliable benchmarking before, and keeps a dedicated,
version-controlled benchmark target rather than ad hoc timing scripts.

**Why:** each test file gets its own executable target instead of being
bundled into a single test binary. This trades link/build time (many small
binaries instead of one) for isolation: a crash or hang in one test file
can't take down or block the rest, and you can build/run exactly the test
you care about (`ninja test_lane_changes_unit`) without rebuilding
everything else. The comment banners (`#StatisticManager test====...===`)
are informal section dividers rather than anything CMake-meaningful — they
exist purely to make this fairly repetitive, manually-maintained list
scannable as it's grown over time (visible in the slightly inconsistent
spacing between some `add_executable`/`target_link_libraries` pairs, a sign
this list has been extended incrementally rather than regenerated).

**Usage:** Notice `test_signals_unit` (line 114) — that's the test for
`TrafficLight` (see `learn/03-traffic-lights.md`); `test_gridlock_stress`
(line 111) and `test_lane_changes_unit` (line 122) are the ones most
directly probing the "junction overlap" and "reactive gridlock" limitations
called out in the syllabus — reading those test files alongside
`learn/07-simulation-core.md` is a good way to see exactly what "reactive"
gridlock handling means in practice.

**Check yourself:**
- Why give every test file its own executable target instead of one combined `all_tests` binary?
- `route_bench` links `traffic_core` just like the test executables do — what does that tell you about what `route_bench` is actually measuring?
