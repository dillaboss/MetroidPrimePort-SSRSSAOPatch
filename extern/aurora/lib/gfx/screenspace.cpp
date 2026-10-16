#include "screenspace.hpp"

#include "../logging.hpp"
#include "../webgpu/gpu.hpp"
#include "../webgpu/gpu_prof.hpp"
#include "frame.hpp"
#include "recording.hpp"

#include <aurora/gfx.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <string>

// Screen-space ambient occlusion and reflections over the opaque world, as drawn so far.
//
// Neither the GameCube's materials nor the EFB carry normals, so both rebuild a view-space
// position from the depth buffer (the projection's frustum and the depth range the world draws
// in) and a normal from the neighbouring depths, taking on each axis the neighbour nearer in
// depth so that silhouettes keep the surface's own slope.
//
//  - Occlusion: 16 samples in the normal's hemisphere (cosine weighted, a golden-angle spiral
//    turned per pixel by a 4x4 ordered pattern), each occluded where the scene is in front
//    of it, weighted by a range check so that distant foreground doesn't darken the background.
//    Faded out towards a distance, where the samples become sub-pixel and noisy.
//  - Reflections: a view-space ray march along the reflected view ray (quadratically spaced
//    steps, jittered per pixel), a hit where the ray passes behind the scene by less than a
//    thickness, refined by bisection. The colour is the frame as it was; the weight is a
//    strength times Schlick's Fresnel, faded at the frame's edges, the ray's far end, and for
//    rays back towards the camera. Only surfaces whose normal is within a set angle of the
//    world's up reflect, so by default floors, water and ice do and rock walls don't.
//
// Both run into work targets (half resolution by default), and one full-screen pass composites
// them with a depth-aware 4x4 filter that also averages out the noise. The display EFB holds
// gamma-encoded colour, which the composite decodes, darkens and blends in linear light and encodes
// again; the HDR scene (GXPortSceneHdr) is linear light already and is worked on as it is, unclipped.
namespace aurora::gfx::screenspace {
namespace {
Module Log("aurora::gfx::screenspace");
using webgpu::g_device;

constexpr auto AoFormat = wgpu::TextureFormat::R8Unorm;
constexpr auto SsrFormat = wgpu::TextureFormat::RGBA16Float;

constexpr const char* CommonSource = R"(
struct Params {
  frustum: vec4f,
  depth: vec4f,
  up: vec4f,
  ao: vec4f,
  ssr: vec4f,
  misc: vec4u, // flags, work width, work height, pad
};

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var depthTex: DEPTH_TYPE;

struct VertexOutput {
  @builtin(position) pos: vec4f,
  @location(0) uv: vec2f,
};

@vertex
fn vs_main(@builtin(vertex_index) i: u32) -> VertexOutput {
  var corners = array<vec2f, 3>(vec2f(-1.0, -1.0), vec2f(3.0, -1.0), vec2f(-1.0, 3.0));
  var out: VertexOutput;
  let c = corners[i];
  out.pos = vec4f(c, 0.0, 1.0);
  out.uv = vec2f(c.x * 0.5 + 0.5, 0.5 - c.y * 0.5);
  return out;
}

fn depth_size() -> vec2i {
  return vec2i(textureDimensions(depthTex));
}

// The linear view depth (positive) at a depth texel; 0 for nothing drawn there (the sky, the
// clear) and for the viewmodel, which draws nearer than the world's depth range.
// Reversed Z: the buffer holds 1 - GX z. Computed from 1 - d, which the buffer holds exactly,
// so distant surfaces keep their precision.
fn view_depth_at(at: vec2i) -> f32 {
  let size = depth_size();
  let raw = textureLoad(depthTex, clamp(at, vec2i(0), size - vec2i(1)), 0);
  if (raw <= 1e-7 || raw > 1.0 - p.depth.z) {
    return 0.0;
  }
  let e = clamp((p.depth.w - 1.0 + raw) / max(p.depth.w - p.depth.z, 1e-6), 0.0, 1.0);
  return p.depth.x * p.depth.y / (p.depth.x + e * (p.depth.y - p.depth.x));
}

// uv: 0..1 over the frame, row 0 the top.
fn view_pos(uv: vec2f, zlin: f32) -> vec3f {
  let ray = vec2f(mix(p.frustum.x, p.frustum.y, uv.x), mix(p.frustum.w, p.frustum.z, uv.y));
  return vec3f(ray * zlin, -zlin);
}

fn project(v: vec3f) -> vec2f {
  let s = v.xy / -v.z;
  return vec2f((s.x - p.frustum.x) / (p.frustum.y - p.frustum.x),
               (p.frustum.w - s.y) / (p.frustum.w - p.frustum.z));
}

// xyz: the view-space position at a depth texel; w: 1 where something is drawn.
fn pos_at(at: vec2i) -> vec4f {
  let z = view_depth_at(at);
  let uv = (vec2f(at) + 0.5) / vec2f(depth_size());
  return vec4f(view_pos(uv, z), select(0.0, 1.0, z > 0.0));
}

// The normal from the neighbouring depths, facing the camera.
fn normal_at(at: vec2i, c: vec3f) -> vec3f {
  let r = pos_at(at + vec2i(1, 0));
  let l = pos_at(at - vec2i(1, 0));
  let d = pos_at(at + vec2i(0, 1));
  let u = pos_at(at - vec2i(0, 1));
  if ((r.w == 0.0 && l.w == 0.0) || (u.w == 0.0 && d.w == 0.0)) {
    return normalize(-c);
  }
  var dx = r.xyz - c;
  if (r.w == 0.0 || (l.w != 0.0 && abs(l.z - c.z) < abs(r.z - c.z))) {
    dx = c - l.xyz;
  }
  // Up the frame is down the rows.
  var dy = u.xyz - c;
  if (u.w == 0.0 || (d.w != 0.0 && abs(d.z - c.z) < abs(u.z - c.z))) {
    dy = c - d.xyz;
  }
  var n = normalize(cross(dx, dy));
  if (dot(n, c) > 0.0) {
    n = -n;
  }
  return n;
}

// A 4x4 ordered (Bayer) pattern: every 4x4 block of the work target holds each of its 16
// values once, so the composite's 4x4 filter averages it out exactly on a flat surface.
fn noise4(px: vec2f) -> f32 {
  let q = vec2u(px) % vec2u(4u);
  var m = array<u32, 16>(0u, 8u, 2u, 10u, 12u, 4u, 14u, 6u, 3u, 11u, 1u, 9u, 15u, 7u, 13u, 5u);
  return (f32(m[q.y * 4u + q.x]) + 0.5) / 16.0;
}

// The depth texel a work-target pixel is computed at.
fn work_texel(pos: vec2f) -> vec2i {
  let size = depth_size();
  let scale = vec2f(size) / vec2f(f32(p.misc.y), f32(p.misc.z));
  return min(vec2i(pos * scale), size - vec2i(1));
}
)";

constexpr const char* AoSource = R"(
const Samples = 16u;

@fragment
fn fs_ao(in: VertexOutput) -> @location(0) vec4f {
  let size = depth_size();
  let at = work_texel(in.pos.xy);
  let c = pos_at(at);
  if (c.w == 0.0) {
    return vec4f(1.0);
  }
  let zlin = -c.z;
  let fade = 1.0 - smoothstep(p.ao.z * 0.6, p.ao.z, zlin);
  if (fade <= 0.0) {
    return vec4f(1.0);
  }
  let n = normal_at(at, c.xyz);
  // A basis around the normal (Duff et al., "Building an Orthonormal Basis, Revisited").
  let s = select(-1.0, 1.0, n.z >= 0.0);
  let a = -1.0 / (s + n.z);
  let b = n.x * n.y * a;
  let t = vec3f(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
  let bt = vec3f(b, s + n.y * n.y * a, -n.y);
  let noise = noise4(in.pos.xy);
  let radius = p.ao.x;
  let origin = c.xyz + n * (p.ao.w * zlin);
  var occlusion = 0.0;
  for (var i = 0u; i < Samples; i++) {
    let fi = f32(i);
    let h = (fi + 0.5) / f32(Samples);
    let phi = fi * 2.39996323 + noise * 6.2831853;
    let sinT = sqrt(h);
    let cosT = sqrt(1.0 - h);
    // More samples close in, where the contact shadows are.
    let k = mix(0.1, 1.0, fract(fi * 0.618034 + noise));
    let dir = t * (cos(phi) * sinT) + bt * (sin(phi) * sinT) + n * cosT;
    let sp = origin + dir * (radius * k * k);
    if (sp.z > -p.depth.x) {
      continue;
    }
    let uv = project(sp);
    if (any(uv < vec2f(0.0)) || any(uv >= vec2f(1.0))) {
      continue;
    }
    let sz = view_depth_at(vec2i(uv * vec2f(size)));
    if (sz > 0.0 && -sz > sp.z) {
      occlusion += smoothstep(0.0, 1.0, radius / abs(zlin - sz));
    }
  }
  let ao = 1.0 - occlusion / f32(Samples);
  return vec4f(mix(1.0, ao, fade), 0.0, 0.0, 1.0);
}
)";

constexpr const char* SsrSource = R"(
@group(0) @binding(2) var frame: texture_2d<f32>;
@group(0) @binding(3) var samp: sampler;

const Steps = 32u;

@fragment
fn fs_ssr(in: VertexOutput) -> @location(0) vec4f {
  let size = depth_size();
  let at = work_texel(in.pos.xy);
  let c = pos_at(at);
  if (c.w == 0.0) {
    return vec4f(0.0);
  }
  let n = normal_at(at, c.xyz);
  if (dot(n, p.up.xyz) < p.ssr.w) {
    return vec4f(0.0);
  }
  let v = normalize(c.xyz);
  let r = reflect(v, n);
  let nv = clamp(dot(n, -v), 0.0, 1.0);
  // Schlick's Fresnel from a reflectance of 0.2: polished stone, ice, still water.
  let weight = p.ssr.x * (0.2 + 0.8 * pow(1.0 - nv, 5.0));
  // Rays back towards the camera leave the frame through its near side; fade them out.
  let facing = 1.0 - smoothstep(0.0, 0.5, r.z);
  if (weight * facing <= 0.002) {
    return vec4f(0.0);
  }
  let zlin = -c.z;
  let origin = c.xyz + n * (0.01 + 0.002 * zlin);
  let len = p.ssr.y;
  let noise = noise4(in.pos.xy);
  var prevT = 0.0;
  var hitT = -1.0;
  for (var i = 1u; i <= Steps; i++) {
    let f = (f32(i) - noise) / f32(Steps);
    let t = len * f * f;
    let q = origin + r * t;
    if (q.z > -p.depth.x) {
      break;
    }
    let uv = project(q);
    if (any(uv < vec2f(0.0)) || any(uv >= vec2f(1.0))) {
      break;
    }
    let sz = view_depth_at(vec2i(uv * vec2f(size)));
    // The scene in front of the ray: a hit when the ray is no further behind it than the
    // surface's assumed thickness (plus the step, which may have skipped through it).
    if (sz > 0.0 && -sz > q.z && (-sz - q.z) < p.ssr.z + (t - prevT)) {
      hitT = t;
      break;
    }
    prevT = t;
  }
  if (hitT < 0.0) {
    return vec4f(0.0);
  }
  var lo = prevT;
  var hi = hitT;
  for (var k = 0; k < 5; k++) {
    let mid = 0.5 * (lo + hi);
    let q = origin + r * mid;
    let sz = view_depth_at(vec2i(project(q) * vec2f(size)));
    if (sz > 0.0 && -sz > q.z) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  let uv = project(origin + r * hi);
  let edge = min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y));
  let fade = smoothstep(0.0, 0.08, edge) * (1.0 - smoothstep(0.6, 1.0, hi / len));
  let color = textureSampleLevel(frame, samp, clamp(uv, vec2f(0.0), vec2f(1.0)), 0.0);
  return vec4f(color.rgb, clamp(weight * facing * fade, 0.0, 1.0));
}
)";

constexpr const char* CompositeSource = R"(
@group(0) @binding(2) var frame: texture_2d<f32>;
@group(0) @binding(3) var aoTex: texture_2d<f32>;
@group(0) @binding(4) var ssrTex: texture_2d<f32>;

// The frame is the HDR scene (RGBA16F linear light) rather than the 8-bit display EFB.
const Hdr = HDR_DOMAIN;

// The EFB's piecewise sRGB curve, as volfog.cpp has it.
fn srgb_enc(c: vec3f) -> vec3f {
  let l = clamp(c, vec3f(0.0), vec3f(1.0));
  return select(1.055 * pow(l, vec3f(1.0 / 2.4)) - 0.055, 12.92 * l, l <= vec3f(0.0031308));
}

fn srgb_dec(c: vec3f) -> vec3f {
  let e = clamp(c, vec3f(0.0), vec3f(1.0));
  return select(pow((e + 0.055) / 1.055, vec3f(2.4)), e / 12.92, e <= vec3f(0.04045));
}

@fragment
fn fs_composite(in: VertexOutput) -> @location(0) vec4f {
  let fsize = vec2i(textureDimensions(frame));
  let f = textureLoad(frame, min(vec2i(floor(in.pos.xy)), fsize - vec2i(1)), 0);
  let flags = p.misc.x;
  let size = depth_size();
  let zc = view_depth_at(min(vec2i(in.uv * vec2f(size)), size - vec2i(1)));
  if (zc <= 0.0) {
    if ((flags & 4u) != 0u) {
      return vec4f(1.0, 1.0, 1.0, f.a);
    }
    if ((flags & 8u) != 0u) {
      return vec4f(0.0, 0.0, 0.0, f.a);
    }
    return f;
  }
  // A depth-aware 4x4 filter over the work targets: it upsamples them and averages out their
  // 4x4 noise pattern (see noise4), without bleeding across depth edges.
  let wsize = vec2i(textureDimensions(aoTex));
  let wscale = vec2f(size) / vec2f(wsize);
  let wc = in.uv * vec2f(wsize) - 0.5;
  let base = vec2i(floor(wc)) - vec2i(1);
  var aoSum = 0.0;
  var rgbSum = vec3f(0.0);
  var aSum = 0.0;
  var wSum = 0.0;
  for (var y = 0; y < 4; y++) {
    for (var x = 0; x < 4; x++) {
      let q = clamp(base + vec2i(x, y), vec2i(0), wsize - vec2i(1));
      let qd = view_depth_at(min(vec2i((vec2f(q) + 0.5) * wscale), size - vec2i(1)));
      if (qd <= 0.0) {
        continue;
      }
      let w = exp(-abs(qd - zc) / (0.03 * zc));
      aoSum += textureLoad(aoTex, q, 0).r * w;
      let s = textureLoad(ssrTex, q, 0);
      rgbSum += s.rgb * s.a * w;
      aSum += s.a * w;
      wSum += w;
    }
  }
  var ao = 1.0;
  var reflAlpha = 0.0;
  var refl = vec3f(0.0);
  if (wSum > 1e-6) {
    if ((flags & 1u) != 0u) {
      ao = pow(clamp(aoSum / wSum, 0.0, 1.0), p.ao.y);
    }
    if ((flags & 2u) != 0u && aSum > 1e-6) {
      refl = rgbSum / aSum;
      reflAlpha = clamp(aSum / wSum, 0.0, 1.0);
    }
  }
  if ((flags & 4u) != 0u) {
    return vec4f(vec3f(ao), f.a);
  }
  if ((flags & 8u) != 0u) {
    return vec4f(refl * reflAlpha, f.a);
  }
  if (ao > 0.999 && reflAlpha < 0.001) {
    return f;
  }
  if (Hdr) {
    return vec4f(mix(f.rgb * ao, refl, reflAlpha), f.a);
  }
  let lit = mix(srgb_dec(f.rgb) * ao, srgb_dec(refl), reflAlpha);
  return vec4f(srgb_enc(lit), f.a);
}
)";

// Which frame a recorded pass works on: the 8-bit display EFB, or the HDR scene (GXPortSceneHdr).
enum class Domain : uint32_t { Ldr, Hdr };
constexpr size_t DomainCount = 2;

// What a recorded pass works on, as volfog.cpp has it: the LDR frame's targets are read at encode (the
// ones its frame was recorded against); the HDR scene's are captured when the pass is recorded, so a
// later mode switch, frame reset or resize can't change the domain or the textures it reads.
struct Capture {
  Domain domain = Domain::Ldr;
  webgpu::TextureWithSampler color;    // the HDR scene (MSAA when samples > 1)
  webgpu::TextureWithSampler resolved; // its resolve target
  webgpu::TextureWithSampler depth;
};

// Recorded passes. A frame records at most two (the sharp scan window draws the world twice), and a
// slot is live from its record to its frame's encode, at most FrameSlotCount frames.
constexpr size_t RecordedSlots = 8;
constexpr size_t MaxPassesPerFrame = 2;
static_assert(MaxPassesPerFrame * detail::FrameSlotCount <= RecordedSlots,
              "screen-space slots would be overwritten while pending");

struct State {
  EncoderTaskId task = InvalidEncoderTask;
  // One uniform buffer per slot: a frame's passes share one encoder and one submit, and a queue write
  // lands before the submit, so a shared buffer would give each pass the last one's params.
  std::array<wgpu::Buffer, RecordedSlots> uniforms;
  wgpu::Sampler sampler;
  // The occlusion and reflection passes draw into the work targets, so they follow only the depth
  // buffer's sample count.
  wgpu::RenderPipeline ao;
  wgpu::RenderPipeline ssr;
  wgpu::BindGroupLayout aoLayout;
  wgpu::BindGroupLayout ssrLayout;
  uint32_t workSamples = 0;
  // The composite for the frame's format and sample count, one per domain.
  struct Composite {
    wgpu::RenderPipeline pipeline;
    wgpu::BindGroupLayout layout;
    wgpu::TextureFormat format = wgpu::TextureFormat::Undefined;
    uint32_t samples = 0;
  };
  std::array<Composite, DomainCount> composite;
  // The frame as it was, one per domain (the HDR one RGBA16F, never narrowed to 8 bits); remade when
  // the frame's size or format changes.
  struct Frame {
    wgpu::Texture texture;
    wgpu::TextureView view;
    wgpu::TextureFormat format = wgpu::TextureFormat::Undefined;
    uint32_t width = 0;
    uint32_t height = 0;
  };
  std::array<Frame, DomainCount> frame;
  // The work targets, remade when their size changes.
  wgpu::TextureView aoView;
  wgpu::TextureView ssrView;
  uint32_t workWidth = 0;
  uint32_t workHeight = 0;
};
State g_state;

std::array<Params, RecordedSlots> g_recorded;
std::array<Capture, RecordedSlots> g_recordedCapture;
uint32_t g_nextSlot = 0;

wgpu::ShaderModule make_module(const char* label, const char* source, uint32_t samples, Domain domain) {
  std::string code = std::string(CommonSource) + source;
  const auto replace = [&code](const std::string& token, const char* with) {
    const size_t at = code.find(token);
    if (at != std::string::npos) {
      code.replace(at, token.size(), with);
    }
  };
  replace("DEPTH_TYPE", samples > 1 ? "texture_depth_multisampled_2d" : "texture_depth_2d");
  replace("HDR_DOMAIN", domain == Domain::Hdr ? "true" : "false");
  wgpu::ShaderSourceWGSL wgsl{};
  wgsl.code = code.c_str();
  const wgpu::ShaderModuleDescriptor descriptor{.nextInChain = &wgsl, .label = label};
  return g_device.CreateShaderModule(&descriptor);
}

wgpu::BindGroupLayoutEntry uniform_entry() {
  return wgpu::BindGroupLayoutEntry{
      .binding = 0,
      .visibility = wgpu::ShaderStage::Fragment,
      .buffer = {.type = wgpu::BufferBindingType::Uniform, .minBindingSize = sizeof(Params)},
  };
}

wgpu::BindGroupLayoutEntry depth_entry(uint32_t samples) {
  return wgpu::BindGroupLayoutEntry{
      .binding = 1,
      .visibility = wgpu::ShaderStage::Fragment,
      .texture = {.sampleType = wgpu::TextureSampleType::Depth,
                  .viewDimension = wgpu::TextureViewDimension::e2D,
                  .multisampled = samples > 1},
  };
}

wgpu::BindGroupLayoutEntry float_entry(uint32_t binding, bool filterable) {
  return wgpu::BindGroupLayoutEntry{
      .binding = binding,
      .visibility = wgpu::ShaderStage::Fragment,
      .texture = {.sampleType =
                      filterable ? wgpu::TextureSampleType::Float : wgpu::TextureSampleType::UnfilterableFloat,
                  .viewDimension = wgpu::TextureViewDimension::e2D},
  };
}

template <size_t N>
wgpu::RenderPipeline make_pipeline(const char* label, const char* source, const char* entry, uint32_t samples,
                                   Domain domain, wgpu::TextureFormat target, uint32_t targetSamples,
                                   const std::array<wgpu::BindGroupLayoutEntry, N>& entries,
                                   wgpu::BindGroupLayout& layoutOut) {
  const auto module = make_module(label, source, samples, domain);
  const wgpu::BindGroupLayoutDescriptor layoutDescriptor{
      .label = label,
      .entryCount = entries.size(),
      .entries = entries.data(),
  };
  layoutOut = g_device.CreateBindGroupLayout(&layoutDescriptor);
  const wgpu::PipelineLayoutDescriptor pipelineLayoutDescriptor{
      .label = label,
      .bindGroupLayoutCount = 1,
      .bindGroupLayouts = &layoutOut,
  };
  const auto pipelineLayout = g_device.CreatePipelineLayout(&pipelineLayoutDescriptor);
  const wgpu::ColorTargetState colorTarget{.format = target, .writeMask = wgpu::ColorWriteMask::All};
  const wgpu::FragmentState fragment{
      .module = module,
      .entryPoint = entry,
      .targetCount = 1,
      .targets = &colorTarget,
  };
  const wgpu::RenderPipelineDescriptor descriptor{
      .label = label,
      .layout = pipelineLayout,
      .vertex = {.module = module, .entryPoint = "vs_main"},
      .primitive = {.topology = wgpu::PrimitiveTopology::TriangleList},
      .multisample = {.count = targetSamples, .mask = UINT32_MAX},
      .fragment = &fragment,
  };
  return g_device.CreateRenderPipeline(&descriptor);
}

void ensure_shared() {
  if (g_state.sampler) {
    return;
  }
  for (size_t i = 0; i < g_state.uniforms.size(); ++i) {
    const wgpu::BufferDescriptor bufferDescriptor{
        .label = "Screen Space Uniforms",
        .usage = wgpu::BufferUsage::Uniform | wgpu::BufferUsage::CopyDst,
        .size = sizeof(Params),
    };
    g_state.uniforms[i] = g_device.CreateBuffer(&bufferDescriptor);
  }
  const wgpu::SamplerDescriptor samplerDescriptor{
      .label = "Screen Space Sampler",
      .addressModeU = wgpu::AddressMode::ClampToEdge,
      .addressModeV = wgpu::AddressMode::ClampToEdge,
      .magFilter = wgpu::FilterMode::Linear,
      .minFilter = wgpu::FilterMode::Linear,
  };
  g_state.sampler = g_device.CreateSampler(&samplerDescriptor);
}

void ensure_work_pipelines(uint32_t samples) {
  if (g_state.ao && g_state.workSamples == samples) {
    return;
  }
  g_state.ao = make_pipeline("Screen Space AO", AoSource, "fs_ao", samples, Domain::Ldr, AoFormat, 1,
                             std::array{uniform_entry(), depth_entry(samples)}, g_state.aoLayout);
  g_state.ssr = make_pipeline("Screen Space Reflections", SsrSource, "fs_ssr", samples, Domain::Ldr, SsrFormat, 1,
                              std::array{uniform_entry(), depth_entry(samples), float_entry(2, true),
                                         wgpu::BindGroupLayoutEntry{
                                             .binding = 3,
                                             .visibility = wgpu::ShaderStage::Fragment,
                                             .sampler = {.type = wgpu::SamplerBindingType::Filtering},
                                         }},
                              g_state.ssrLayout);
  g_state.workSamples = samples;
}

void ensure_composite(Domain domain, wgpu::TextureFormat format, uint32_t samples) {
  auto& composite = g_state.composite[static_cast<size_t>(domain)];
  if (composite.pipeline && composite.format == format && composite.samples == samples) {
    return;
  }
  composite.pipeline = make_pipeline(domain == Domain::Hdr ? "Screen Space Composite (HDR)" : "Screen Space Composite",
                                     CompositeSource, "fs_composite", samples, domain, format, samples,
                                     std::array{uniform_entry(), depth_entry(samples), float_entry(2, false),
                                                float_entry(3, false), float_entry(4, false)},
                                     composite.layout);
  composite.format = format;
  composite.samples = samples;
}

wgpu::TextureView make_target(const char* label, uint32_t width, uint32_t height, wgpu::TextureFormat format,
                              wgpu::TextureUsage usage) {
  const wgpu::TextureDescriptor descriptor{
      .label = label,
      .usage = usage,
      .size = {width, height, 1},
      .format = format,
  };
  return g_device.CreateTexture(&descriptor).CreateView();
}

void ensure_frame(Domain domain, uint32_t width, uint32_t height, wgpu::TextureFormat format) {
  auto& frame = g_state.frame[static_cast<size_t>(domain)];
  if (frame.texture && frame.width == width && frame.height == height && frame.format == format) {
    return;
  }
  frame.width = width;
  frame.height = height;
  frame.format = format;
  const wgpu::TextureDescriptor descriptor{
      .label = domain == Domain::Hdr ? "Screen Space Frame Copy (HDR)" : "Screen Space Frame Copy",
      .usage = wgpu::TextureUsage::TextureBinding | wgpu::TextureUsage::CopyDst,
      .size = {width, height, 1},
      .format = format,
  };
  frame.texture = g_device.CreateTexture(&descriptor);
  frame.view = frame.texture.CreateView();
}

void ensure_work_targets(uint32_t workWidth, uint32_t workHeight) {
  if (g_state.aoView && g_state.workWidth == workWidth && g_state.workHeight == workHeight) {
    return;
  }
  g_state.workWidth = workWidth;
  g_state.workHeight = workHeight;
  constexpr auto workUsage = wgpu::TextureUsage::RenderAttachment | wgpu::TextureUsage::TextureBinding;
  g_state.aoView = make_target("Screen Space AO", workWidth, workHeight, AoFormat, workUsage);
  g_state.ssrView = make_target("Screen Space Reflections", workWidth, workHeight, SsrFormat, workUsage);
}

// The HDR targets a slot reads, checked as one whole, as volfog.cpp does: the scene format throughout,
// the depth buffer's size and sample count those of the colour, a resolve target when multisampled.
// Anything else fails closed (the frame is left alone).
bool hdr_capture_valid(const Capture& c, uint32_t& samples) {
  if (!c.color.texture || !c.color.view || !c.depth.texture || !c.depth.view ||
      c.color.format != webgpu::SceneHdrFormat || c.color.texture.GetFormat() != webgpu::SceneHdrFormat) {
    return false;
  }
  samples = c.color.texture.GetSampleCount();
  if (c.depth.texture.GetSampleCount() != samples || c.depth.size.width != c.color.size.width ||
      c.depth.size.height != c.color.size.height) {
    return false;
  }
  if (samples > 1) {
    return c.resolved.texture && c.resolved.view && c.resolved.format == webgpu::SceneHdrFormat &&
           c.resolved.texture.GetFormat() == webgpu::SceneHdrFormat && c.resolved.texture.GetSampleCount() == 1 &&
           c.resolved.size.width == c.color.size.width && c.resolved.size.height == c.color.size.height;
  }
  return true;
}

wgpu::BindGroup make_group(const char* label, const wgpu::BindGroupLayout& layout, const wgpu::BindGroupEntry* entries,
                           size_t count) {
  const wgpu::BindGroupDescriptor descriptor{
      .label = label,
      .layout = layout,
      .entryCount = count,
      .entries = entries,
  };
  return g_device.CreateBindGroup(&descriptor);
}

void draw_pass(const wgpu::CommandEncoder& cmd, const char* label, const wgpu::TextureView& view,
               const wgpu::TextureView& resolve, const wgpu::RenderPipeline& pipeline, const wgpu::BindGroup& group,
               const wgpu::Color& clear) {
  // Every pixel is written without blending, so the target need not be loaded first.
  const wgpu::RenderPassColorAttachment attachment{
      .view = view,
      .resolveTarget = resolve,
      .loadOp = wgpu::LoadOp::Clear,
      .storeOp = wgpu::StoreOp::Store,
      .clearValue = clear,
  };
  const wgpu::RenderPassDescriptor descriptor{
      .label = label,
      .colorAttachmentCount = 1,
      .colorAttachments = &attachment,
      .timestampWrites = webgpu::gpu_prof::pass_writes(label),
  };
  const auto pass = cmd.BeginRenderPass(&descriptor);
  pass.SetPipeline(pipeline);
  pass.SetBindGroup(0, group);
  pass.Draw(3);
  pass.End();
}

void encode(const EncoderTaskContext& ctx, const wgpu::CommandEncoder& cmd, const void* payload, size_t payloadSize,
            void*) {
  uint32_t slot = 0;
  if (payloadSize != sizeof(slot)) {
    return;
  }
  std::memcpy(&slot, payload, sizeof(slot));
  Params params = g_recorded[slot % g_recorded.size()];
  const Capture& capture = g_recordedCapture[slot % g_recordedCapture.size()];
  if ((params.flags & (FlagAo | FlagSsr)) == 0) {
    return;
  }
  const Domain domain = capture.domain;
  const bool hdr = domain == Domain::Hdr;
  uint32_t hdrSamples = 1;
  if (hdr && !hdr_capture_valid(capture, hdrSamples)) {
    return;
  }
  // The targets this frame was recorded against (a resize may have made newer ones since).
  const auto efb = webgpu::efb_targets();
  const auto& source = hdr ? (hdrSamples > 1 ? capture.resolved : capture.color) : webgpu::present_source();
  const auto& target = hdr ? capture.color : efb.color;
  const auto& resolveTarget = hdr ? capture.resolved : efb.resolved;
  const auto& depth = hdr ? capture.depth : efb.depth;
  const uint32_t samples =
      hdr ? hdrSamples : (webgpu::g_graphicsConfig.msaaSamples > 1 ? webgpu::g_graphicsConfig.msaaSamples : 1);
  const auto format = hdr ? webgpu::SceneHdrFormat : webgpu::g_graphicsConfig.surfaceConfiguration.format;
  const uint32_t width = source.size.width;
  const uint32_t height = source.size.height;
  if (width == 0 || height == 0 || !source.texture || !target.view || !depth.view) {
    return;
  }
  const bool half = (params.flags & FlagHalfRes) != 0;
  const uint32_t workWidth = half ? std::max((width + 1) / 2, 1u) : width;
  const uint32_t workHeight = half ? std::max((height + 1) / 2, 1u) : height;
  ensure_shared();
  ensure_work_pipelines(samples);
  ensure_composite(domain, format, samples);
  ensure_frame(domain, width, height, format);
  ensure_work_targets(workWidth, workHeight);
  const auto& composite = g_state.composite[static_cast<size_t>(domain)];
  const auto& frame = g_state.frame[static_cast<size_t>(domain)];
  const wgpu::Buffer uniforms = g_state.uniforms[slot % g_state.uniforms.size()];
  params.size[0] = workWidth;
  params.size[1] = workHeight;
  ctx.queue.WriteBuffer(uniforms, 0, &params, sizeof(params));

  const wgpu::TexelCopyTextureInfo copySource{.texture = source.texture};
  const wgpu::TexelCopyTextureInfo copyTarget{.texture = frame.texture};
  const wgpu::Extent3D copySize{width, height, 1};
  cmd.CopyTextureToTexture(&copySource, &copyTarget, &copySize);

  const wgpu::BindGroupEntry uniformEntry{.binding = 0, .buffer = uniforms, .size = sizeof(Params)};
  const wgpu::BindGroupEntry depthEntry{.binding = 1, .textureView = depth.view};
  if ((params.flags & FlagAo) != 0) {
    const std::array entries{uniformEntry, depthEntry};
    const auto group = make_group("Screen Space AO", g_state.aoLayout, entries.data(), entries.size());
    draw_pass(cmd, "SSAO", g_state.aoView, {}, g_state.ao, group, {1.0, 1.0, 1.0, 1.0});
  }
  if ((params.flags & FlagSsr) != 0) {
    const std::array entries{
        uniformEntry,
        depthEntry,
        wgpu::BindGroupEntry{.binding = 2, .textureView = frame.view},
        wgpu::BindGroupEntry{.binding = 3, .sampler = g_state.sampler},
    };
    const auto group = make_group("Screen Space Reflections", g_state.ssrLayout, entries.data(), entries.size());
    draw_pass(cmd, "SSR", g_state.ssrView, {}, g_state.ssr, group, {0.0, 0.0, 0.0, 0.0});
  }
  const std::array entries{
      uniformEntry,
      depthEntry,
      wgpu::BindGroupEntry{.binding = 2, .textureView = frame.view},
      wgpu::BindGroupEntry{.binding = 3, .textureView = g_state.aoView},
      wgpu::BindGroupEntry{.binding = 4, .textureView = g_state.ssrView},
  };
  const auto group = make_group("Screen Space Composite", composite.layout, entries.data(), entries.size());
  draw_pass(cmd, "Screen-space composite", target.view, samples > 1 ? resolveTarget.view : wgpu::TextureView{},
            composite.pipeline, group, {0.0, 0.0, 0.0, 0.0});
}
} // namespace

bool supported() noexcept { return webgpu::g_hasCoreFeatures; }

bool ensure_task() {
  if (!supported()) {
    return false;
  }
  if (g_state.task == InvalidEncoderTask) {
    g_state.task = register_encoder_task_type(EncoderTaskDescriptor{.label = "Screen Space", .callback = encode});
    if (g_state.task == InvalidEncoderTask) {
      Log.warn("could not register the screen-space task");
      return false;
    }
  }
  return true;
}

bool record(const Params& params) {
  if (g_state.task == InvalidEncoderTask || (params.flags & (FlagAo | FlagSsr)) == 0) {
    return false;
  }
  // The domain is the pass's, here at the FIFO, as the volumetric fog takes it. The HDR scene on but
  // this pass not its own (offscreen, a colour pass) has no target these know: fail closed.
  const bool hdr = scene_hdr_pass_active();
  if (!hdr && scene_hdr_active()) {
    return false;
  }
  const auto& size = hdr ? webgpu::g_sceneHdr.size : webgpu::present_source().size;
  if (size.width == 0 || size.height == 0) {
    return false;
  }
  const uint32_t slot = g_nextSlot++ % g_recorded.size();
  g_recorded[slot] = params;
  g_recordedCapture[slot] = hdr ? Capture{.domain = Domain::Hdr,
                                          .color = webgpu::g_sceneHdr,
                                          .resolved = webgpu::g_sceneHdrResolved,
                                          .depth = webgpu::g_depthBuffer}
                                : Capture{};
  record_encoder_task(g_state.task, &slot, sizeof(slot));
  return true;
}

void shutdown() {
  const auto task = g_state.task;
  g_state = {};
  g_recordedCapture = {};
  if (task != InvalidEncoderTask) {
    unregister_encoder_task_type(task);
  }
}
} // namespace aurora::gfx::screenspace
