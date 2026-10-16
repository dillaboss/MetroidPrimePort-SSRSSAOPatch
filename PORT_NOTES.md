# Port notes

Open follow-ups and non-obvious facts for the port. Closed items are removed; see
git history. Build, run and test instructions are in `docs/NATIVE_PORT.md`.

## Open follow-ups

### Touch overlay: a short tap can be missed

SDL's virtual joystick is state-sampling, not event-queueing: a press and release
that both land between two updates leave only the release, so a quick tap on A or
Start can do nothing, mostly while a frame is stalled. Fixing it means latching a
press until the game has sampled it, and releasing the latch on an update the port
does not control. Documented in `platform/include/touch_pad.h`. Also: the cached
virtual-pad pointer would not survive a full `SDL_Quit()`, so same-process activity
recreation would need explicit close/detach/reset (not reachable today:
`SDLActivity` exits the process on a second creation).

### Prompt art coverage

- `0xe14dc493` (yellow "C" badge, a C-stick prompt) is in `platform/port_prompts.cpp`
  but no reachable screen has been seen drawing it; not the pause menu, map or HUD.
- The front end's B prompt and the HUD hint-memo prompt have not been photographed,
  and nothing prompt-related was checked on a device.
- Finding a texture hash: `MP_DUMP_TEXTURES=1` dumps only textures that have no
  replacement yet and writes the images, so a candidate can be looked at. Deleting a
  table row does not remove the static per-device replacement files in `textures/`
  (`PortTextures` registers them by filename); delete the files too.

### Archipelago

- The real world is `UltiNaruto/MetroidAPrime` (the 404 on 2026-09-27 came from a
  misspelled `MetroidAPPrime`; the repository is reachable). The location join is
  still verified only against `tools/ap-world-fixture/Locations.py` (retail's 100
  locations, synthetic AP ids from 50310000); map the real table with
  `tools/make_ap_config.py --strict`.
- `MP_AP_RESET_STATE=1` is the manual way to discard progress for a new game or an
  older save on the same slot and seed (the card exposes no save identity).
- `MP_RANDO_DUMP` takes precedence over `MP_RANDO_SEED` and returns before placement,
  so setting both looks like the seed was ignored. A seed run wants
  `MP_RANDO_SWEEP=1` and no `MP_RANDO_DUMP`.

### Validation gaps

- Windows runtime and full-game traversal were never validated beyond the CI
  startup check. Full-game ThreadSanitizer is blocked by reports in uninstrumented
  GLib/libdbus/nod startup code; the 205 GPU-free FIFO/GX tests pass under TSan.
- Retail save compatibility (MSB-first bit fields, big-endian CRC, legacy native CRC
  accepted) has had no directed round trip with a retail GameCube save.
- A Windows CI note: the port's own tests that need an unloaded process re-run the
  test binary (`fork`/`execv`, `CreateProcessA`); winsock2.h needs `NOMINMAX` first.
- A full eight-world sweep (~37000 frames) can end in
  `VK_ERROR_OUT_OF_DEVICE_MEMORY` loading the front end's first texture. The dump is
  already complete by then, so it is cosmetic for the sweep.
- Capture limits: the screenshot path (`F12`, console `shot`, via
  `aurora::request_screenshot`) does not include Aurora's ImGui layer, so the F1
  overlay cannot be verified from a capture; HUD memos are captured normally.

### Packaging

- Flathub: screenshots of the running game would show Nintendo's game, so none can
  ship; the app id is already `io.github.odrannnn.metroidprimeport`.
- The licence (`LICENSE` MIT over this project's own work; `NOTICE` excludes `src/`
  and `include/`) does not answer what may be done with a working copy of the
  decompiled game code; that remains a question for the copyright holder.

## Non-obvious facts

- **Audit of what a package contains**: no `.iso`/`.gcm`/`.wbfs`; the APK carries 162
  texture assets derived from Kenney's CC0 Input Prompts (attribution in
  `tools/prompt_icons/README.md`) and the third-party licence texts under `assets/`.
- **`OSDisableInterrupts` is a no-op on PC.** Anything the guest serialised with it
  needs a real mutex or ordering (DVD/ARAM transfer counters, `CDSPStream`
  `xec_readsPending`: publish state before starting an async read, drop completions
  for reads the stream no longer owns).
- **Plain `char` is unsigned on ARM**, so `-1` sentinels read back as 255. CMake passes
  `-fsigned-char` (`mp_signed_char`) and `platform/compat.h` asserts it.
- **`AIGetDMAStartAddr` truncates to 32 bits**; use the port's 64-bit accessor
  (`platform/ai_dma.cpp`). The AI DMA callback runs on the main thread because the
  guest mixer is not thread-safe. `MP_DISABLE_AI_AUDIO=1` isolates this path.
- **Aurora defers ARQ completion callbacks until `ARQPoll`**; loops that spin on a DMA
  token (`CARAMToken::UpdateAllDMAs`) must pump it.
- **`GXSetDrawSync`/`GXReadDrawSync` are real FIFO-ordered tokens.** The skinned-model
  workspace is freed when its token reads back, so an echoing shim corrupts draws.
- **Array sizes and endianness**: `GXSetArray` needs real byte sizes (0 draws nothing);
  runtime-generated arrays are host-native (`le=true`), and `ClearArray` is needed
  when a workspace pointer is reused each frame.
- **AGSC sample directory**: the disc form ends in a 4-byte `0xFFFFFFFF` terminator,
  so the ADPCM info blocks start at `(count - 1) * entrySize + 4`; `extraData` offsets
  are rebased onto the larger native `SDIR_DATA`. MusyX 2.0.0's `sndPopGroup` can
  leave voices referencing a popped group's samples, so group buffers live exactly
  as long as the group is pushed (`f2888a72`).
- **Wayland**: the game EFB stays at its configured 640x480 with
  `VISetFrameBufferScale(1)` and Aurora scales to the swapchain (otherwise the title
  background vanishes at fractional scales); compositor vsync is off in favour of an
  absolute 60 Hz deadline. SDL's Wayland backend can hang in `SDL_ShowWindow`
  (libdecor); see `docs/NATIVE_PORT.md`. Unattended runs on a Wayland session need
  `DISPLAY`, `XAUTHORITY` and `SDL_VIDEODRIVER=x11`.
- **Lock-on** (`CPlayer::WithinOrbitScreenBox`/`Ellipse`) compares screen position
  against fixed 640x480 tweak coordinates; they are scaled by the viewport size,
  otherwise the zone sits left of the reticle in widescreen.
- **Aspect** (`MP_ASPECT=4:3|16:9|window`): the game recomputes the render-mode width
  each frame (`CGraphics::PortResizeFrameBuffer`) and refreshes `CCameraManager`'s
  aspect; `AURORA_VIEWPORT_FIT` letterboxes instead of stretching.
- **Frame pacing**: the simulation is fixed-step at 60 Hz; uncapped (`F10`) only
  raises the presentation rate. The reported presented rate and throughput (wait
  excluded) differ at the cap and agree uncapped.
- **HD textures** (`MP_TEXTURES=<dir>`): Aurora's `tex1_<w>x<h>[_m]_<texhash>[_<tluthash>]_<format>.dds|.png`
  convention, `$` wildcards allowed. Dolphin packs use the same layout and
  `XXH64(data, size, 0)`, but the hashed size and TLUT handling are unconfirmed against
  a real pack.
- **`assets/initial_pipeline_cache.db`** holds machine-independent pipeline
  descriptions, copied beside the executable and merged into each user's cache.
- **Debug flags**: `MP_FAST_BOOT=1` skips the front end into a new game, so a
  Continue walk (`MP_SMOKE_CONTINUE`) needs it off. `MP_SKIP_CUTSCENES=1` fast-forwards
  cutscenes without a skip object (the frigate opening has none).
  `MP_SMOKE_SCRIPT`, `MP_SMOKE_SAVE` and the other smoke drivers are in
  `docs/NATIVE_PORT.md`.
- **Port hooks run from `CStateManager::Update`**, which only runs in `kSMT_InGame`;
  anything that must act while a menu is up has to run from `PortSmokeFrame` (main
  loop).
- **Android memory card path**: `CARDInit` fills an empty card path from
  `MP_USER_PATH` (unset on Android) or the working directory (not writable there).
  The port falls back to `SDL_GetAndroidInternalStoragePath()` and logs
  `memory card: storing under <path>`; start any device investigation of a card that
  the front end cannot identify from that line.
- **Android release signing**: release builds refuse the debug key unless asked
  (`tools/android_apk.sh` opts in with a stderr note unless `--strict-signing`);
  naming no signing config at all produces an uninstallable `app-release-unsigned.apk`.
- **A green Gradle build proves nothing by itself**: it can report success without
  compiling the C++. Check the object file postdates the edit.
- **Saves**: Prime saves at a save station (no save in the pause menu).
  `CARDCreate` alone leaves an 8192-byte zero file with CRC 0, which the game rightly
  calls corrupt; that is not a port bug.
- **Licensing**: Aurora and the vendored MusyX snapshot are MIT; the decompiled game
  and its assets stay Nintendo's, so ship no assets and require the user's disc. The
  earlier GPL recomp path (DolRecomp / ModernGekko) is kept separately and not linked.
