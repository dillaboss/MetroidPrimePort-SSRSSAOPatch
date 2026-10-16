# Metroid Prime — native port

A native build of **Metroid Prime** for Linux, Windows and Android (plus an
experimental, untested-on-hardware macOS build), compiled
from the [PrimeDecomp](https://github.com/PrimeDecomp/prime) decompilation and
rendered through Aurora (SDL3/WebGPU). There's no emulator involved. Supported
discs: USA 1.00, 1.01 and 1.02, and PAL.

**No game content is included.** You need your own copy of the game: the port
reads it from a disc image and never ships any of its assets. This repository
ships no disc image, and neither does any package or release built from it.

## What it adds over the console release

- **Remastered assets:** — (Experimental) import models and effects from your own Metroid Prime
  Remastered dump (F1 > Remastered)
- **Widescreen:** with an edge-to-edge HUD, and **uncapped frame rate**
- **Graphics:** FOV, MSAA, anisotropic filtering, HUD scale, arm cannon offset,
  and toggles to hide the helmet and visor effects
- **Controls:** mouse and twin-stick aim, gyro, full rebinding and presets
- **Optional tweaks:** Fast Morph, Spring Ball, Toggle
  Lock-On, Sticky Charge...
- **Skippable cutscenes**
- **Save states** and **quick resume** after quitting
- **Saves:** import from and export to USA/PAL GameCube, Wii Trilogy and Dolphin
- **Photo mode** with free-cam and high-resolution capture
- **Map and tracker:** reveal the map, plus item, scan and room tracking
- **Dolphin texture packs**, including HD texture replacements
- **Discord Rich Presence** on desktop, and a **touch overlay** on Android
- The **F1 overlay** has every setting, with search; the common ones are also in
  pause > Options
- And more..!

## Randomizer and Archipelago Multiworld

- **Randomizer** — Generate your own randomizer games
- **Archipelago multiworld** — Generate a game in archipelago with the [Metroid Prime APWorld](https://github.com/UltiNaruto/MetroidAPrime) and host it. Open F1 > Archipelago, enter the server (`host:port`), your
  slot name and the password if any, and connect; no seed file is needed. Each seed and slot gets its own separate save, checks made
  offline are sent on the next connect, chat with command support, DeathLink.
  
## Downloads

Releases are published on the
[GitHub releases page](https://github.com/Odrannnn/MetroidPrimePort/releases).
Every build needs your own disc image; none of them contains one.

**Quick start:** download the build for your platform (Windows zip, Linux
AppImage or tarball, Android APK), start it, and pick your disc image when it
asks. It remembers the choice. Supported discs are the GameCube USA 1.00, 1.01
and 1.02 (`GM8E01`) and PAL (`GM8P01`) releases, as `.iso`, `.gcm`, `.rvz`,
`.wbfs`, `.ciso` or `.nkit`. The Japanese and Korean releases, the Wii Trilogy and
Remastered are not run as the base game. Press **F1** for every setting.

## Building

Needs CMake 3.25+, Ninja and a C++20 toolchain. Aurora and MusyX are vendored, so
a normal clone is enough; Aurora fetches its pinned dependencies on the first
configure.

```sh
cmake -S . -B build/native -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DAURORA_ENABLE_TESTS=OFF -DMP_BUILD_TESTS=ON
cmake --build build/native -j 4
ctest --test-dir build/native -L port --output-on-failure
```

On Windows, run from an MSVC developer shell and add
`-DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl`. CI builds and tests
Linux (GCC 14 and Clang 18) on each push to `port` and each pull request; Windows (clang-cl) and macOS (arm64,
experimental) run weekly and can be started by hand from the Actions tab. See `.github/workflows/`.
CI configures with `-DAURORA_ENABLE_TESTS=ON`, which adds Aurora's GPU-free
FIFO/GX tests; turn it on to run the same suite locally.

Configure downloads about a dozen pinned dependencies (checked by SHA-256). On
a machine without access to GitHub, clone them yourself and point CMake at the
copies with `-DFETCHCONTENT_SOURCE_DIR_<NAME>=<dir>`, the way
`flatpak/io.github.odrannnn.metroidprimeport.yml` does.

### Android

```sh
tools/make_android_keystore.sh          # once; writes android/keystore.properties
tools/android_apk.sh :app:assembleRelease
```

## Running

```sh
./build/native/metroid_prime_port "/path/to/Metroid Prime (USA) (v1.00).iso"
```

## Troubleshooting

- **Where are the logs?** `metroid_prime_port.log` in the user folder (F1 >
  System shows its path); the previous run's is `metroid_prime_port.old.log`. On
  Android a copy goes to `Documents/MetroidPrime/`. Attach it to a bug report.
- **"unsupported disc"**: the image is not one of the revisions above (see the
  log for the disc id it found). Re-dump it, or check you picked the GameCube disc.
- **Stuck at startup on a GNOME Wayland session**: run with
  `SDL_VIDEODRIVER=x11`. The port does this itself when `DISPLAY` is set; see
  `docs/NATIVE_PORT.md`.
- **Settings in a bad state**: F1 > System > Settings > **Reset configuration...**
  backs up and resets every setting and mapping; saves are not touched.

## Packaging

`tools/make_appimage.sh` builds a self-contained AppImage and
`tools/make_flatpak.sh` a Flatpak. Neither bundles the disc. See
`docs/NATIVE_PORT.md` for what each one expects from the host.

## Credits and licensing

This is a fork of the [PrimeDecomp/prime](https://github.com/PrimeDecomp/prime)
decompilation, which the game logic is built from. Aurora (`extern/aurora`) and
MusyX (`extern/musyx`) are MIT-licensed vendored snapshots, and the button
prompt icons come from Kenney's Input Prompts pack (CC0), vendored under
`tools/prompt_icons`. Their licences are kept alongside them and are copied into
release packages.

`LICENSE` covers this project's own work — the platform layer, the build and
packaging scripts, and the Android and Flatpak integration. It does **not** cover
the game code under `src/` and `include/`, which comes from the decompilation
and remains Nintendo's. See `NOTICE`.

Metroid Prime is a trademark of Nintendo. This project is unaffiliated with
Nintendo and Retro Studios, and ships no game data.

## Documentation

| Document | What it covers |
|---|---|
| `docs/SETTINGS.md` | Every control, setting and environment variable |
| `docs/NATIVE_PORT.md` | The detailed reference: build options, smoke-test hooks, platform notes |
| `docs/MODS.md` | HD texture packs and the mods folder |
| `docs/RANDOMIZER.md` | The built-in randomizer |
| `docs/ARCHIPELAGO.md` | The Archipelago multiworld client |
| `docs/CONSOLE.md` | The debug console (`MP_CONSOLE`) |
| `docs/DEBUGGING.md` | Headless debugging with `tools/mprig.py` |
| `docs/RELEASING.md` | What a release contains and must carry |
| `docs/ANDROID_BUILD_PROBE.md` | The Android port's status and build |
| `docs/REMASTERED_EFFECTS.md` | Importing Remastered particle effects |
| `docs/FRAME_INTERPOLATION.md`, `docs/HIGH_FPS_AUDIT.md` | High frame rate design notes |
| `docs/decomp/` | Notes inherited from the decompilation project |
| `PORT_NOTES.md` | Open follow-ups and non-obvious facts |

## AI assistance

Parts of this port were written with AI coding assistants, under my oversight.
