#pragma once

#include <cstdint>

// Port extension: screen-space ambient occlusion and screen-space reflections, run on the
// finished EFB between two passes (see GXPortScreenSpace). Both read only the depth buffer and
// the colour drawn so far: the GameCube's surfaces have no normals or roughness to hand over, so
// normals are rebuilt from depth and how much a surface reflects is a setting, not a material.
namespace aurora::gfx::screenspace {
constexpr uint32_t FlagAo = 1;       // ambient occlusion
constexpr uint32_t FlagSsr = 2;      // reflections
constexpr uint32_t FlagShowAo = 4;   // debug: the frame is the occlusion alone
constexpr uint32_t FlagShowSsr = 8;  // debug: the frame is the reflections alone
constexpr uint32_t FlagHalfRes = 16; // occlusion and reflections at half resolution

// The task's uniform, word for word. View space is GX's: x right, y up, z towards the camera.
struct Params {
  float frustum[4]; // left, right, bottom, top at a view depth of 1
  float depth[4];   // near, far, and the GX z range the world draws in (min, max)
  float up[4];      // xyz: the world's up in view space
  float ao[4];      // radius (world units), intensity (an exponent), fade-out distance, bias
  float ssr[4];     // strength, ray length (world units), thickness, the least dot(normal, up)
  uint32_t flags;
  uint32_t size[2]; // set by the task: the work targets' width and height
  uint32_t pad;
};
static_assert(sizeof(Params) == 24 * 4);

// Whether the device can run the passes: they read the depth buffer texel by texel, which
// WebGPU's compatibility mode (some OpenGL ES devices) does not allow.
bool supported() noexcept;
// Registers the encoder task (game thread); false if it could not be, or if not supported().
bool ensure_task();
// Records the passes from the FIFO processor (GX_AURORA_PORT_SCREEN_SPACE), once ensure_task has
// returned true. False if nothing was recorded.
bool record(const Params& params);
void shutdown();
} // namespace aurora::gfx::screenspace
