// Sprite pipeline for the illustrated map.
//
// Loads the project's own transparent PNGs from assets/sprites/ and exposes
// ready-to-draw <img>/canvas handles. Unlike the reference (AI JPGs on a magenta
// key), our art is already transparent, so there is NO keying/watermark step —
// just load, cache, and expose. Every sprite is OPTIONAL: the renderer falls
// back to procedural drawing for anything missing or failed to load.
//
// The illustrated map only needs ground + trees; the rest (buildings, vehicles,
// lights) are declared here so later PRs get them for free.
"use strict";

export const Sprites = (() => {
    // Logical name -> file. Buildings/vehicles/lights are pre-registered for
    // later PRs but the static map only draws "ground-*" and "tree-*".
    const FILES = {
        "ground-1": "grass-1.png",
        "ground-2": "grass-2.png",
        "ground-3": "grass-3.png",
        "tree-1": "tree-1.png",
        "tree-2": "tree-2.png",
        "tree-3": "tree-3.png",
        "tree-4": "tree-4.png",
        "building-1": "building-1.png",
        "building-2": "building-2.png",
        "building-3": "building-3.png",
    };

    const store = {};        // name -> { img, w, h }
    let loadedPromise = null;

    function loadOne(name, file) {
        return new Promise((resolve) => {
            const img = new Image();
            img.onload = () => {
                store[name] = { img, w: img.naturalWidth, h: img.naturalHeight };
                resolve();
            };
            img.onerror = () => resolve(); // missing sprite -> procedural fallback
            img.src = "../assets/sprites/" + file;
        });
    }

    // Resolve once all image loads have settled (success or failure).
    function load() {
        if (!loadedPromise) {
            loadedPromise = Promise.all(
                Object.entries(FILES).map(([n, f]) => loadOne(n, f))
            );
        }
        return loadedPromise;
    }

    // { img, w, h } or null if the sprite is unavailable.
    function get(name) {
        return store[name] || null;
    }

    // Deterministic pick from a family ("tree" -> tree-1..4) keyed by a number,
    // so scenery is identical on every load.
    function pick(family, count, seed) {
        return get(family + "-" + (1 + (seed % count)));
    }

    return { load, get, pick };
})();
