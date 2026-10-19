#pragma once

#include "port_echoes_probe.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Echoes' animations, decoded and carried onto Prime's Samus (see port_echoes_probe.h). Pure: bytes
// in, poses out, no engine types, so it can be tested and checked on its own.
namespace PortEchoesAnim {

struct Quat {
  float w = 1.f, x = 0.f, y = 0.f, z = 0.f;
};
struct Vec3 {
  float x = 0.f, y = 0.f, z = 0.f;
};

Quat Mul(const Quat& a, const Quat& b); // Hamilton: a then... applied as a * b
Quat Conj(const Quat& q);
Quat Normalized(const Quat& q);
Vec3 Rotate(const Quat& q, const Vec3& v);
Quat Slerp(const Quat& a, const Quat& b, float t);
float AngleBetween(const Quat& a, const Quat& b); // degrees, sign of the quaternion ignored

// --- The animation set: which ANIM each named animation plays ---------------------------------

struct AnimationEntry {
  std::string name;
  std::vector< uint32_t > anims; // the ANIM resources its meta-animation plays, first leaf first
};

// The ANCS animation set that starts at `offset` (where ParseCharacterSet stopped).
bool ParseAnimationSet(const uint8_t* data, size_t size, size_t offset,
                       std::vector< AnimationEntry >& out, std::string& error);

// --- ANIM ---------------------------------------------------------------------------------------

struct Channel {
  uint32_t bone = 0;
  std::vector< Quat > rotations;    // per key; empty when the channel has none
  std::vector< Vec3 > translations; // per key
  std::vector< Vec3 > scales;       // per key
};

struct Clip {
  uint32_t version = 0; // 0 uncompressed, 2 compressed
  float duration = 0.f;
  float interval = 0.f;
  bool looping = false;
  uint32_t rootBone = 0;
  std::vector< uint32_t > keyFrames; // the frame each key sits at, ascending, key 0 at frame 0
  std::vector< Channel > channels;
  // Compressed clips: how the bitstream's length came out against what the header asks for. A
  // header or descriptor misread shows up here before anywhere else.
  uint64_t bitsUsed = 0;
  uint64_t bitsAvailable = 0;
  uint32_t keyBitmapKeys = 0;  // frames marked as keys after frame 0
  uint32_t bitmapFrames = 0;   // frames the key bitmap covers
  uint32_t descriptorKeys = 0; // the largest key count a channel declares
  // The channel table layout the bitstream fitted (see ParseClip), and how many others fitted as
  // well as it.
  uint32_t boneIdBytes = 0;
  uint32_t wordsAfterBitmap = 0;
  uint32_t layoutTies = 0;
};

bool ParseClip(const uint8_t* data, size_t size, Clip& out, std::string& error);

// A channel's value at `time` (seconds), between the keys around it.
struct Sample {
  bool hasRotation = false;
  bool hasTranslation = false;
  Quat rotation;
  Vec3 translation;
};
Sample SampleChannel(const Clip& clip, const Channel& channel, float time);

// --- Poses ------------------------------------------------------------------------------------

// How a rotation key is taken: as the bone's whole local rotation (replacing the bind local
// rotation), or as a turn from the bind pose (bind local * key).
enum class RotationMode { Replace, FromBind };
// How a translation key is taken: as the bone's offset from its parent, or added to the bind one.
enum class TranslationMode { Replace, Add };

struct PoseOptions {
  RotationMode rotation = RotationMode::Replace;
  TranslationMode translation = TranslationMode::Replace;
};

// Every bone's skeleton-space place and orientation for `clip` at `time` on Echoes' skeleton,
// indexed like skeleton.bones.
struct WorldPose {
  std::vector< Vec3 > positions;
  std::vector< Quat > orientations;
};
WorldPose EchoesPose(const PortEchoesProbe::Skeleton& skeleton, const Clip& clip, float time,
                     const PoseOptions& options);

// The same pose carried onto Prime's skeleton (no bind rotations): each Prime bone's local rotation
// (parent-relative), indexed like prime.bones. Bones Echoes lacks keep the rest orientation of
// their parent (a local identity). Also fills `world` with Prime's skeleton-space pose.
std::vector< Quat > RetargetToPrime(const PortEchoesProbe::Skeleton& echoes,
                                    const WorldPose& echoesPose,
                                    const PortEchoesProbe::Skeleton& prime, WorldPose& world);

// The check written into the report: decodes the named clips and measures them.
struct ClipInput {
  std::string name;
  std::vector< uint8_t > data; // empty when the ANIM was not found
};
std::string BuildAnimationCheck(const PortEchoesProbe::Skeleton& echoes,
                                const PortEchoesProbe::Skeleton& prime,
                                const std::vector< ClipInput >& clips);

} // namespace PortEchoesAnim
