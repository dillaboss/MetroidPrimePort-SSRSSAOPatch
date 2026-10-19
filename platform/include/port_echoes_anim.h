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
//
// Echoes' skeletons carry bind rotations; Prime's bones rest unrotated. A pose is worked out on
// Echoes' skeleton as each bone's "unbound" orientation Q (its world orientation with the bind
// rotation taken out, so the bind pose is all identity, as on Prime) and its place; carried onto
// Prime's skeleton, Q is the orientation a Prime bone of the same name takes.
//
// How Echoes' keys apply is what the report measures; a Convention is one candidate:
//   Prime        the key is the bone's local rotation as on Prime's unrotated bones (no bind);
//   Absolute     the key is the bone's whole local rotation in the bind frames (replaces the bind
//                local rotation);
//   BindThenKey  bind local * key;  KeyThenBind  key * bind local;
// each with the key conjugated or not, and translation keys read in the parent's unrotated frame
// or its bound one. Bones without a key keep the bind pose.
enum class KeyMode { Prime, Absolute, BindThenKey, KeyThenBind };
enum class TranslationFrame { Unrotated, ParentBind };
struct Convention {
  KeyMode mode = KeyMode::Prime;
  bool conjugate = false;
  TranslationFrame translation = TranslationFrame::Unrotated;
};
// Every candidate, in a fixed order (settings refer to them by index).
const std::vector< Convention >& Conventions();
std::string ConventionName(const Convention& c);

// Whether the skeleton's rotations are stored inverted (as world-to-bone): the skeleton's own local
// rotations say so (local = bind * parent^-1 when inverted, parent^-1 * bind when not).
bool BindStoredInverse(const PortEchoesProbe::Skeleton& skeleton);

// A pose on Echoes' skeleton, indexed like skeleton.bones (skeleton space).
struct WorldPose {
  std::vector< Vec3 > positions;
  std::vector< Quat > orientations; // unbound: identity in the bind pose
  std::vector< bool > moved;        // the clip set this bone's offset (a translation key)
};
WorldPose EchoesPose(const PortEchoesProbe::Skeleton& skeleton, const Clip& clip, float time,
                     const Convention& convention);

// The pose carried onto Prime's skeleton, indexed like prime.bones: each bone's local rotation
// (parent-relative, what the pose builder takes), skeleton-space orientation and place, and, for
// bones whose Echoes counterpart was moved by a translation key, the offset from the parent in the
// parent's frame (unrotated layout offsets otherwise). Bones Echoes lacks keep their parent's
// orientation (a local identity).
struct PrimePose {
  std::vector< Quat > locals;
  std::vector< Quat > orientations;
  std::vector< Vec3 > positions;
  std::vector< bool > hasOffset;
  std::vector< Vec3 > offsets;
};
PrimePose RetargetToPrime(const PortEchoesProbe::Skeleton& echoes, const WorldPose& pose,
                          const PortEchoesProbe::Skeleton& prime);

// --- Choosing the convention from the clips themselves ------------------------------------------

struct NamedClip {
  std::string name; // "ready" in it: a standing clip; walk/jog/run/strafe: moving
  const Clip* clip = nullptr;
};
struct ConventionScore {
  size_t index = 0;   // into Conventions()
  float score = 0.f;  // lower is better
  float ground = 0.f; // how far the lowest foot is from the floor, averaged over the clips (m)
  float head = 0.f;   // standing clips: head height against the bind pose's (m)
  float skate = 0.f;  // moving clips: the planted foot's slide against the body's travel (ratio)
  int broken = 0;     // samples with the body folded (head below hips, feet above knees)
};
// Best first.
std::vector< ConventionScore > RankConventions(const PortEchoesProbe::Skeleton& echoes,
                                               const std::vector< NamedClip >& clips);

// --- The report ---------------------------------------------------------------------------------

struct ClipInput {
  std::string name;
  std::vector< uint8_t > data; // empty when the ANIM was not found
};
std::string BuildAnimationCheck(const PortEchoesProbe::Skeleton& echoes,
                                const PortEchoesProbe::Skeleton& prime,
                                const std::vector< ClipInput >& clips);

} // namespace PortEchoesAnim
