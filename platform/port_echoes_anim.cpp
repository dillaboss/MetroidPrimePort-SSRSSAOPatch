// Echoes animation decoding and retargeting onto Prime's Samus (see port_echoes_anim.h).
//
// ANIM version 2 (compressed), as Prime's own CFBStreamedCompression reads it, with Echoes'
// differences from the Retro modding wiki: no EVNT id, a u16 where Prime has a u32, a scale
// multiplier, no repeated channel count, and channel descriptors whose rotation, translation and
// scale parts are each present only with a non-zero key count. The bitstream is big-endian words
// read from their least significant bit; per key and channel: a W sign bit and three rotation
// deltas, then three translation deltas, then three scale deltas, each part only when the channel
// has it. Key 0 is the descriptors' initial values at frame 0; the key bitmap marks the frames of
// the keys after it. Rotations are X, Y, Z = sin(value * (pi/2) / divisor), W from the unit length
// with the sign bit.
//
// ANIM version 0 (Echoes' uncompressed layout): durations as CCharAnimTime (f32, u32), key count,
// root bone, a 100-entry bone-to-channel table, then per-part channel tables and key arrays
// (scale, rotation as W X Y Z, translation), each bone-major.
#include "port_echoes_anim.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>

namespace PortEchoesAnim {
namespace {

using PortEchoesProbe::Bone;
using PortEchoesProbe::Skeleton;

class Reader {
public:
  Reader(const uint8_t* data, size_t size, size_t pos = 0)
  : mData(data), mSize(size), mPos(std::min(pos, size)) {}
  bool ok() const { return mOk; }
  size_t pos() const { return mPos; }
  size_t left() const { return mSize - mPos; }
  bool Has(size_t n) const { return mOk && mSize - mPos >= n; }
  void Fail() { mOk = false; }
  uint8_t U8() {
    if (!Has(1)) {
      Fail();
      return 0;
    }
    return mData[mPos++];
  }
  uint16_t U16() {
    if (!Has(2)) {
      Fail();
      return 0;
    }
    const uint16_t v = uint16_t((mData[mPos] << 8) | mData[mPos + 1]);
    mPos += 2;
    return v;
  }
  int16_t S16() { return int16_t(U16()); }
  uint32_t U32() {
    if (!Has(4)) {
      Fail();
      return 0;
    }
    const uint32_t v = (uint32_t(mData[mPos]) << 24) | (uint32_t(mData[mPos + 1]) << 16) |
                       (uint32_t(mData[mPos + 2]) << 8) | uint32_t(mData[mPos + 3]);
    mPos += 4;
    return v;
  }
  float F32() {
    const uint32_t bits = U32();
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
  }
  std::string String() {
    std::string s;
    while (true) {
      if (!Has(1) || s.size() > 1024) {
        Fail();
        return s;
      }
      const char c = char(mData[mPos++]);
      if (c == '\0') {
        return s;
      }
      s.push_back(c);
    }
  }
  void Skip(size_t n) {
    if (!Has(n)) {
      Fail();
      return;
    }
    mPos += n;
  }
  uint32_t Count(size_t minBytes, uint32_t limit) {
    const uint32_t n = U32();
    if (!mOk || n > limit || (minBytes != 0 && uint64_t(n) * minBytes > left())) {
      Fail();
      return 0;
    }
    return n;
  }

private:
  const uint8_t* mData;
  size_t mSize;
  size_t mPos;
  bool mOk = true;
};

// The compressed bitstream: big-endian words, each read from its least significant bit.
class Bits {
public:
  Bits(const uint8_t* data, size_t words) : mData(data), mWords(words) {}
  bool ok() const { return mOk; }
  uint64_t used() const { return mUsed; }
  uint32_t Unsigned(uint32_t count) {
    uint32_t value = 0;
    for (uint32_t i = 0; i < count; ++i) {
      const size_t word = size_t(mUsed / 32);
      if (word >= mWords) {
        mOk = false;
        return 0;
      }
      const uint8_t* p = mData + word * 4;
      const uint32_t w =
          (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
      value |= ((w >> (mUsed % 32)) & 1u) << i;
      ++mUsed;
    }
    return value;
  }
  int32_t Signed(uint32_t count) {
    if (count == 0) {
      return 0;
    }
    uint32_t v = Unsigned(count);
    if (count < 32 && (v & (1u << (count - 1))) != 0) {
      v |= ~0u << count;
    }
    return int32_t(v);
  }

private:
  const uint8_t* mData;
  size_t mWords;
  uint64_t mUsed = 0;
  bool mOk = true;
};

constexpr float kPi = 3.14159265358979f;

void Line(std::string& out, const char* format, ...) {
  char buf[2048];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  out += buf;
  out += '\n';
}

bool SkipMetaAnim(Reader& in, std::vector< uint32_t >& anims, int depth) {
  if (depth > 32) {
    in.Fail();
    return false;
  }
  const uint32_t type = in.U32();
  switch (type) {
  case 0: // Play: the primitive (ANIM id, index, name) and a start time
    anims.push_back(in.U32());
    in.U32();
    in.String();
    in.Skip(8);
    break;
  case 1: // Blend
  case 2: // PhaseBlend
    SkipMetaAnim(in, anims, depth + 1);
    SkipMetaAnim(in, anims, depth + 1);
    in.Skip(5); // weight, character-space flag
    break;
  case 3: { // Random
    const uint32_t n = in.Count(8, 1000);
    for (uint32_t i = 0; i < n && in.ok(); ++i) {
      SkipMetaAnim(in, anims, depth + 1);
      in.U32();
    }
    break;
  }
  case 4: { // Sequence
    const uint32_t n = in.Count(4, 1000);
    for (uint32_t i = 0; i < n && in.ok(); ++i) {
      SkipMetaAnim(in, anims, depth + 1);
    }
    break;
  }
  default:
    in.Fail();
    break;
  }
  return in.ok();
}

// The index of each bone in the skeleton, by id.
std::map< uint32_t, size_t > IndexById(const Skeleton& s) {
  std::map< uint32_t, size_t > m;
  for (size_t i = 0; i < s.bones.size(); ++i) {
    m[s.bones[i].id] = i;
  }
  return m;
}

// Bones ordered parents first.
std::vector< size_t > ParentsFirst(const Skeleton& s) {
  const auto index = IndexById(s);
  std::vector< int > depth(s.bones.size(), -1);
  std::function< int(size_t, int) > depthOf = [&](size_t i, int guard) -> int {
    if (depth[i] >= 0) {
      return depth[i];
    }
    if (guard > 200) {
      return 0;
    }
    const auto p = index.find(s.bones[i].parent);
    depth[i] = p == index.end() || p->second == i ? 0 : depthOf(p->second, guard + 1) + 1;
    return depth[i];
  };
  std::vector< size_t > order(s.bones.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
    depthOf(i, 0);
  }
  std::stable_sort(order.begin(), order.end(),
                   [&](size_t a, size_t b) { return depth[a] < depth[b]; });
  return order;
}

Quat ToQuat(const float q[4]) { return Normalized(Quat{q[0], q[1], q[2], q[3]}); }
Vec3 ToVec(const float v[3]) { return Vec3{v[0], v[1], v[2]}; }
Vec3 Sub(const Vec3& a, const Vec3& b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 Add(const Vec3& a, const Vec3& b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
float Length(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

} // namespace

Quat Mul(const Quat& a, const Quat& b) {
  return Quat{
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quat Conj(const Quat& q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

Quat Normalized(const Quat& q) {
  const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!(n > 1e-8f)) {
    return Quat{};
  }
  return Quat{q.w / n, q.x / n, q.y / n, q.z / n};
}

Vec3 Rotate(const Quat& q, const Vec3& v) {
  const Quat p{0.f, v.x, v.y, v.z};
  const Quat r = Mul(Mul(q, p), Conj(q));
  return Vec3{r.x, r.y, r.z};
}

Quat Slerp(const Quat& a, const Quat& bIn, float t) {
  Quat b = bIn;
  float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < 0.f) {
    b = Quat{-b.w, -b.x, -b.y, -b.z};
    d = -d;
  }
  if (d > 0.9995f) {
    return Normalized(Quat{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                           a.z + (b.z - a.z) * t});
  }
  const float th = std::acos(std::min(d, 1.f));
  const float s = std::sin(th);
  const float wa = std::sin((1.f - t) * th) / s;
  const float wb = std::sin(t * th) / s;
  return Quat{a.w * wa + b.w * wb, a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb};
}

float AngleBetween(const Quat& a, const Quat& b) {
  // From the difference rotation's parts, which stays precise for small angles (an arc cosine of
  // the dot product loses them to rounding).
  const Quat d = Mul(Conj(Normalized(a)), Normalized(b));
  const float v = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  return 2.f * std::atan2(v, std::fabs(d.w)) * 180.f / kPi;
}

bool ParseAnimationSet(const uint8_t* data, size_t size, size_t offset,
                       std::vector< AnimationEntry >& out, std::string& error) {
  out.clear();
  Reader in(data, size, offset);
  const uint16_t version = in.U16();
  const uint32_t count = in.Count(5, 5000);
  if (!in.ok() || version == 0 || version > 8) {
    error = "no animation set where the characters end";
    return false;
  }
  for (uint32_t i = 0; i < count; ++i) {
    AnimationEntry e;
    e.name = in.String();
    if (!SkipMetaAnim(in, e.anims, 0)) {
      error = "animation " + std::to_string(i) + " ('" + e.name + "') could not be read";
      return !out.empty();
    }
    out.push_back(std::move(e));
  }
  return true;
}

bool ParseClip(const uint8_t* data, size_t size, Clip& out, std::string& error) {
  out = {};
  Reader in(data, size);
  out.version = in.U32();
  if (!in.ok()) {
    error = "empty";
    return false;
  }
  if (out.version == 0) {
    out.duration = in.F32();
    in.U32();
    out.interval = in.F32();
    in.U32();
    const uint32_t keys = in.U32();
    out.rootBone = in.U32();
    const uint32_t boneSlots = in.Count(1, 1000);
    std::vector< uint8_t > boneToChannel(boneSlots);
    for (uint8_t& v : boneToChannel) {
      v = in.U8();
    }
    const auto table = [&in]() {
      const uint32_t n = in.Count(1, 1000);
      std::vector< uint8_t > t(n);
      for (uint8_t& v : t) {
        v = in.U8();
      }
      return t;
    };
    const std::vector< uint8_t > rotTable = table();
    const std::vector< uint8_t > transTable = table();
    const std::vector< uint8_t > scaleTable = table();
    const uint32_t scaleKeys = in.Count(12, 1u << 20);
    std::vector< Vec3 > scales(scaleKeys);
    for (Vec3& v : scales) {
      v = Vec3{in.F32(), in.F32(), in.F32()};
    }
    const uint32_t rotKeys = in.Count(16, 1u << 20);
    std::vector< Quat > rots(rotKeys);
    for (Quat& q : rots) {
      q = Quat{in.F32(), in.F32(), in.F32(), in.F32()};
    }
    const uint32_t transKeys = in.Count(12, 1u << 20);
    std::vector< Vec3 > trans(transKeys);
    for (Vec3& v : trans) {
      v = Vec3{in.F32(), in.F32(), in.F32()};
    }
    if (!in.ok() || keys == 0 || keys > 100000) {
      error = "uncompressed ANIM runs past the end";
      return false;
    }
    for (uint32_t k = 0; k < keys; ++k) {
      out.keyFrames.push_back(k);
    }
    out.looping = true;
    for (size_t bone = 0; bone < boneToChannel.size(); ++bone) {
      const uint8_t c = boneToChannel[bone];
      if (c == 0xFF) {
        continue;
      }
      Channel ch;
      ch.bone = uint32_t(bone);
      const auto part = [&](const std::vector< uint8_t >& t) -> int {
        return c < t.size() && t[c] != 0xFF ? t[c] : -1;
      };
      const int r = part(rotTable);
      const int tr = part(transTable);
      const int sc = part(scaleTable);
      for (uint32_t k = 0; k < keys; ++k) {
        if (r >= 0 && size_t(r) * keys + k < rots.size()) {
          ch.rotations.push_back(Normalized(rots[size_t(r) * keys + k]));
        }
        if (tr >= 0 && size_t(tr) * keys + k < trans.size()) {
          ch.translations.push_back(trans[size_t(tr) * keys + k]);
        }
        if (sc >= 0 && size_t(sc) * keys + k < scales.size()) {
          ch.scales.push_back(scales[size_t(sc) * keys + k]);
        }
      }
      out.channels.push_back(std::move(ch));
    }
    return true;
  }
  if (out.version != 2) {
    error = "ANIM version " + std::to_string(out.version) + " is not one this reads";
    return false;
  }

  in.U32(); // scratch size
  in.U16();
  out.duration = in.F32();
  out.interval = in.F32();
  out.rootBone = in.U32();
  out.looping = in.U32() != 0;
  const uint32_t rotDivisor = in.U32();
  const float transMult = in.F32();
  const float scaleMult = in.F32();
  const uint32_t headerChannels = in.U32();
  in.U32();
  const uint32_t bitmapBits = in.Count(0, 1u << 20);
  out.bitmapFrames = bitmapBits;
  const uint32_t bitmapWords = (bitmapBits + 31) / 32;
  std::vector< uint32_t > bitmap(bitmapWords);
  for (uint32_t& w : bitmap) {
    w = in.U32();
  }
  if (!in.ok() || rotDivisor == 0 || !(out.interval > 0.f) || !(out.duration >= 0.f) ||
      headerChannels == 0 || headerChannels > 200) {
    error = "compressed ANIM header runs past the end or is out of range";
    return false;
  }
  // Key frames: frame 0, then each frame the bitmap marks.
  out.keyFrames.push_back(0);
  for (uint32_t f = 1; f < bitmapBits; ++f) {
    if ((bitmap[f / 32] >> (f % 32)) & 1u) {
      out.keyFrames.push_back(f);
    }
  }
  out.keyBitmapKeys = uint32_t(out.keyFrames.size() - 1);
  const uint32_t increments = out.keyBitmapKeys;

  struct Part {
    uint16_t keys = 0;
    int16_t initial[3] = {};
    uint8_t bits[3] = {};
  };
  struct Desc {
    uint32_t bone = 0;
    Part rot, trans, scale;
  };
  // The channel table's layout: how many words sit between the bitmap and the table, and how wide
  // a bone id is. Sources disagree for Echoes, so each candidate is read and the one whose
  // bitstream comes out the length its keys need is taken.
  struct Layout {
    uint32_t wordsAfterBitmap;
    uint32_t idBytes;
    std::vector< Desc > descs;
    size_t streamStart = 0;
    uint64_t needed = 0;
    uint64_t available = 0;
    uint32_t descriptorKeys = 0;
    bool fits = false;
  };
  std::vector< Layout > layouts = {{1, 1, {}}, {1, 4, {}}, {0, 1, {}},
                                   {0, 4, {}}, {2, 1, {}}, {2, 4, {}}};
  const size_t tableStart = in.pos();
  for (Layout& l : layouts) {
    Reader r(data, size, tableStart);
    for (uint32_t i = 0; i < l.wordsAfterBitmap; ++i) {
      r.U32();
    }
    l.descs.resize(headerChannels);
    std::vector< bool > seen(256, false);
    bool plausible = true;
    const auto readPart = [&r, &plausible, bitmapBits](Part& p) {
      p.keys = r.U16();
      if (p.keys != 0) {
        plausible = plausible && p.keys <= bitmapBits;
        for (int i = 0; i < 3; ++i) {
          p.initial[i] = r.S16();
          p.bits[i] = r.U8();
          plausible = plausible && p.bits[i] <= 32;
        }
      }
    };
    for (Desc& d : l.descs) {
      d.bone = l.idBytes == 1 ? r.U8() : r.U32();
      plausible = plausible && d.bone < 256 && !seen[d.bone];
      if (d.bone < 256) {
        seen[d.bone] = true;
      }
      readPart(d.rot);
      readPart(d.trans);
      readPart(d.scale);
    }
    if (!r.ok() || !plausible) {
      continue;
    }
    uint64_t bitsPerKey = 0;
    for (const Desc& d : l.descs) {
      l.descriptorKeys =
          std::max< uint32_t >({l.descriptorKeys, d.rot.keys, d.trans.keys, d.scale.keys});
      if (d.rot.keys != 0) {
        bitsPerKey += 1u + d.rot.bits[0] + d.rot.bits[1] + d.rot.bits[2];
      }
      if (d.trans.keys != 0) {
        bitsPerKey += uint64_t(d.trans.bits[0]) + d.trans.bits[1] + d.trans.bits[2];
      }
      if (d.scale.keys != 0) {
        bitsPerKey += uint64_t(d.scale.bits[0]) + d.scale.bits[1] + d.scale.bits[2];
      }
    }
    l.streamStart = r.pos();
    l.needed = bitsPerKey * increments;
    l.available = uint64_t(r.left() / 4) * 32;
    l.fits = l.needed <= l.available;
  }
  const Layout* chosen = nullptr;
  for (const Layout& l : layouts) {
    if (l.fits &&
        (chosen == nullptr || l.available - l.needed < chosen->available - chosen->needed)) {
      chosen = &l;
    }
  }
  if (chosen == nullptr) {
    error = "no channel table layout fits the bitstream (" + std::to_string(headerChannels) +
            " channels, " + std::to_string(increments) + " keys)";
    return false;
  }
  for (const Layout& l : layouts) {
    if (&l != chosen && l.fits && l.available - l.needed == chosen->available - chosen->needed) {
      ++out.layoutTies;
    }
  }
  const std::vector< Desc >& descs = chosen->descs;
  out.boneIdBytes = chosen->idBytes;
  out.wordsAfterBitmap = chosen->wordsAfterBitmap;
  out.descriptorKeys = chosen->descriptorKeys;
  out.bitsAvailable = chosen->available;
  Bits bits(data + chosen->streamStart, size_t(chosen->available / 32));

  struct Running {
    int32_t r[4];
    int32_t t[3];
    int32_t s[3];
  };
  std::vector< Running > run(descs.size());
  out.channels.resize(descs.size());
  const float rq = (kPi / 2.f) / float(rotDivisor);
  const auto emit = [&](size_t i) {
    const Desc& d = descs[i];
    Channel& ch = out.channels[i];
    const Running& v = run[i];
    if (d.rot.keys != 0) {
      Quat q;
      q.x = std::sin(rq * float(v.r[1]));
      q.y = std::sin(rq * float(v.r[2]));
      q.z = std::sin(rq * float(v.r[3]));
      q.w = std::sqrt(std::max(0.f, 1.f - (q.x * q.x + q.y * q.y + q.z * q.z)));
      if (v.r[0] != 0) {
        q.w = -q.w;
      }
      ch.rotations.push_back(q);
    }
    if (d.trans.keys != 0) {
      ch.translations.push_back(
          Vec3{float(v.t[0]) * transMult, float(v.t[1]) * transMult, float(v.t[2]) * transMult});
    }
    if (d.scale.keys != 0) {
      ch.scales.push_back(
          Vec3{float(v.s[0]) * scaleMult, float(v.s[1]) * scaleMult, float(v.s[2]) * scaleMult});
    }
  };
  for (size_t i = 0; i < descs.size(); ++i) {
    out.channels[i].bone = descs[i].bone;
    run[i] =
        Running{{0, descs[i].rot.initial[0], descs[i].rot.initial[1], descs[i].rot.initial[2]},
                {descs[i].trans.initial[0], descs[i].trans.initial[1], descs[i].trans.initial[2]},
                {descs[i].scale.initial[0], descs[i].scale.initial[1], descs[i].scale.initial[2]}};
    emit(i);
  }
  for (uint32_t k = 0; k < increments && bits.ok(); ++k) {
    for (size_t i = 0; i < descs.size(); ++i) {
      const Desc& d = descs[i];
      Running& v = run[i];
      if (d.rot.keys != 0) {
        v.r[0] = int32_t(bits.Unsigned(1));
        for (int c = 0; c < 3; ++c) {
          v.r[c + 1] += bits.Signed(d.rot.bits[c]);
        }
      }
      if (d.trans.keys != 0) {
        for (int c = 0; c < 3; ++c) {
          v.t[c] += bits.Signed(d.trans.bits[c]);
        }
      }
      if (d.scale.keys != 0) {
        for (int c = 0; c < 3; ++c) {
          v.s[c] += bits.Signed(d.scale.bits[c]);
        }
      }
      emit(i);
    }
  }
  out.bitsUsed = bits.used();
  if (!bits.ok()) {
    error = "the bitstream is shorter than its " + std::to_string(increments) + " keys need";
    return false;
  }
  return true;
}

Sample SampleChannel(const Clip& clip, const Channel& channel, float time) {
  Sample s;
  const size_t keys = std::max(channel.rotations.size(), channel.translations.size());
  if (keys == 0 || clip.keyFrames.empty() || !(clip.interval > 0.f)) {
    return s;
  }
  float frame = time / clip.interval;
  if (clip.looping && clip.duration > 0.f) {
    const float frames = clip.duration / clip.interval;
    frame = std::fmod(std::max(frame, 0.f), frames);
  }
  size_t a = 0;
  while (a + 1 < keys && a + 1 < clip.keyFrames.size() && float(clip.keyFrames[a + 1]) <= frame) {
    ++a;
  }
  const size_t b = std::min(a + 1, keys - 1);
  float t = 0.f;
  if (b != a && b < clip.keyFrames.size()) {
    const float span = float(clip.keyFrames[b]) - float(clip.keyFrames[a]);
    t = span > 0.f ? std::clamp((frame - float(clip.keyFrames[a])) / span, 0.f, 1.f) : 0.f;
  }
  if (!channel.rotations.empty()) {
    s.hasRotation = true;
    const Quat& qa = channel.rotations[std::min(a, channel.rotations.size() - 1)];
    const Quat& qb = channel.rotations[std::min(b, channel.rotations.size() - 1)];
    s.rotation = Slerp(qa, qb, t);
  }
  if (!channel.translations.empty()) {
    s.hasTranslation = true;
    const Vec3& va = channel.translations[std::min(a, channel.translations.size() - 1)];
    const Vec3& vb = channel.translations[std::min(b, channel.translations.size() - 1)];
    s.translation =
        Vec3{va.x + (vb.x - va.x) * t, va.y + (vb.y - va.y) * t, va.z + (vb.z - va.z) * t};
  }
  return s;
}

const std::vector< Convention >& Conventions() {
  static const std::vector< Convention > list = [] {
    std::vector< Convention > v;
    for (bool conj : {false, true}) {
      v.push_back({KeyMode::Prime, conj, TranslationFrame::Unrotated});
    }
    for (KeyMode m : {KeyMode::Absolute, KeyMode::BindThenKey, KeyMode::KeyThenBind}) {
      for (bool conj : {false, true}) {
        for (TranslationFrame t : {TranslationFrame::Unrotated, TranslationFrame::ParentBind}) {
          v.push_back({m, conj, t});
        }
      }
    }
    return v;
  }();
  return list;
}

std::string ConventionName(const Convention& c) {
  std::string name;
  switch (c.mode) {
  case KeyMode::Prime:
    name = "prime";
    break;
  case KeyMode::Absolute:
    name = "absolute";
    break;
  case KeyMode::BindThenKey:
    name = "bind*key";
    break;
  case KeyMode::KeyThenBind:
    name = "key*bind";
    break;
  }
  if (c.conjugate) {
    name += " (key^-1)";
  }
  if (c.mode != KeyMode::Prime) {
    name += c.translation == TranslationFrame::Unrotated ? ", moves unrotated" : ", moves bound";
  }
  return name;
}

namespace {

void SelfCheck(const Skeleton& s, float& asStored, float& asInverse) {
  const auto index = IndexById(s);
  asStored = 0.f;
  asInverse = 0.f;
  for (const Bone& b : s.bones) {
    const auto p = index.find(b.parent);
    const Quat pb = p != index.end() ? ToQuat(s.bones[p->second].rotation) : Quat{};
    const Quat bb = ToQuat(b.rotation);
    const Quat lb = ToQuat(b.localRotation);
    asStored = std::max(asStored, AngleBetween(lb, Mul(Conj(pb), bb)));
    asInverse = std::max(asInverse, AngleBetween(lb, Mul(bb, Conj(pb))));
  }
}

// The bind rotations as bone-to-world rotations, whichever way they are stored.
struct BindFrames {
  std::vector< Quat > world;
  std::vector< Quat > local;
};
BindFrames Binds(const Skeleton& s) {
  const bool inverse = BindStoredInverse(s);
  BindFrames f;
  for (const Bone& b : s.bones) {
    const Quat w = ToQuat(b.rotation);
    const Quat l = ToQuat(b.localRotation);
    f.world.push_back(inverse ? Conj(w) : w);
    f.local.push_back(inverse ? Conj(l) : l);
  }
  return f;
}

float Horizontal(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

int FindBone(const Skeleton& s, const char* name) {
  for (size_t i = 0; i < s.bones.size(); ++i) {
    if (s.bones[i].name == name) {
      return int(i);
    }
  }
  return -1;
}

bool Contains(const std::string& haystack, const char* needle) {
  std::string h = haystack;
  for (char& c : h) {
    c = char(std::tolower(static_cast< unsigned char >(c)));
  }
  return h.find(needle) != std::string::npos;
}

// The frames a clip spans: its keys' last frame, or its duration's.
uint32_t FrameCount(const Clip& clip) {
  uint32_t frames = clip.keyFrames.empty() ? 1 : clip.keyFrames.back() + 1;
  if (clip.interval > 0.f) {
    frames = std::max(frames, uint32_t(std::lround(clip.duration / clip.interval)) + 1);
  }
  return std::min< uint32_t >(frames, 2000);
}

} // namespace

bool BindStoredInverse(const Skeleton& skeleton) {
  float asStored = 0.f;
  float asInverse = 0.f;
  SelfCheck(skeleton, asStored, asInverse);
  return asInverse < asStored;
}

WorldPose EchoesPose(const Skeleton& skeleton, const Clip& clip, float time, const Convention& c) {
  const auto index = IndexById(skeleton);
  const BindFrames bind = Binds(skeleton);
  std::map< uint32_t, const Channel* > channels;
  for (const Channel& ch : clip.channels) {
    channels[ch.bone] = &ch;
  }
  const size_t n = skeleton.bones.size();
  const bool prime = c.mode == KeyMode::Prime;
  WorldPose pose;
  pose.positions.assign(n, Vec3{});
  pose.orientations.assign(n, Quat{});
  pose.moved.assign(n, false);
  std::vector< Quat > world(n, Quat{}); // with the bind rotations in (prime: the same as unbound)
  for (size_t i : ParentsFirst(skeleton)) {
    const Bone& b = skeleton.bones[i];
    const auto p = index.find(b.parent);
    const bool hasParent = p != index.end() && p->second != i;
    const Quat wp = hasParent ? world[p->second] : Quat{};
    const Quat qp = hasParent ? pose.orientations[p->second] : Quat{};
    const Vec3 pp = hasParent ? pose.positions[p->second] : Vec3{};
    Quat local = prime ? Quat{} : bind.local[i];
    Sample s;
    const auto ch = channels.find(b.id);
    if (ch != channels.end()) {
      s = SampleChannel(clip, *ch->second, time);
    }
    if (s.hasRotation) {
      const Quat k = c.conjugate ? Conj(s.rotation) : s.rotation;
      switch (c.mode) {
      case KeyMode::Prime:
      case KeyMode::Absolute:
        local = k;
        break;
      case KeyMode::BindThenKey:
        local = Mul(bind.local[i], k);
        break;
      case KeyMode::KeyThenBind:
        local = Mul(k, bind.local[i]);
        break;
      }
    }
    world[i] = Normalized(Mul(wp, local));
    pose.orientations[i] = prime ? world[i] : Normalized(Mul(world[i], Conj(bind.world[i])));
    if (s.hasTranslation) {
      const bool bound = !prime && c.translation == TranslationFrame::ParentBind;
      pose.positions[i] = Add(pp, Rotate(bound ? wp : qp, s.translation));
      pose.moved[i] = true;
    } else {
      const Vec3 d = hasParent ? Sub(ToVec(b.position), ToVec(skeleton.bones[p->second].position))
                               : ToVec(b.position);
      pose.positions[i] = Add(pp, Rotate(qp, d));
    }
  }
  return pose;
}

PrimePose RetargetToPrime(const Skeleton& echoes, const WorldPose& pose, const Skeleton& prime) {
  const auto index = IndexById(prime);
  const size_t n = prime.bones.size();
  PrimePose out;
  out.locals.assign(n, Quat{});
  out.orientations.assign(n, Quat{});
  out.positions.assign(n, Vec3{});
  out.hasOffset.assign(n, false);
  out.offsets.assign(n, Vec3{});
  std::map< std::string, size_t > byName;
  for (size_t j = 0; j < echoes.bones.size(); ++j) {
    byName[echoes.bones[j].name] = j;
  }
  for (size_t i : ParentsFirst(prime)) {
    const Bone& b = prime.bones[i];
    const auto p = index.find(b.parent);
    const bool hasParent = p != index.end() && p->second != i;
    const Quat qp = hasParent ? out.orientations[p->second] : Quat{};
    const Vec3 pp = hasParent ? out.positions[p->second] : Vec3{};
    const auto e = byName.find(b.name);
    const bool mapped = e != byName.end() && e->second < pose.orientations.size();
    out.orientations[i] = mapped ? pose.orientations[e->second] : qp;
    out.locals[i] = Normalized(Mul(Conj(qp), out.orientations[i]));
    const Vec3 d = hasParent ? Sub(ToVec(b.position), ToVec(prime.bones[p->second].position))
                             : ToVec(b.position);
    if (mapped && pose.moved[e->second]) {
      out.positions[i] = pose.positions[e->second];
      out.offsets[i] = Rotate(Conj(qp), Sub(out.positions[i], pp));
      out.hasOffset[i] = true;
    } else {
      out.positions[i] = Add(pp, Rotate(qp, d));
      out.offsets[i] = d;
    }
  }
  return out;
}

std::vector< ConventionScore > RankConventions(const Skeleton& echoes,
                                               const std::vector< NamedClip >& clips) {
  const int balls[2] = {FindBone(echoes, "L_ball"), FindBone(echoes, "R_ball")};
  const int ankles[2] = {FindBone(echoes, "L_ankle"), FindBone(echoes, "R_ankle")};
  const int knees[2] = {FindBone(echoes, "L_knee"), FindBone(echoes, "R_knee")};
  const int head = FindBone(echoes, "Head_1");
  const int hips = FindBone(echoes, "Skeleton_Root");
  const float bindHead = head >= 0 ? echoes.bones[size_t(head)].position[2] : 0.f;
  std::vector< ConventionScore > scores;
  const auto& conventions = Conventions();
  for (size_t ci = 0; ci < conventions.size(); ++ci) {
    ConventionScore sc;
    sc.index = ci;
    float groundSum = 0.f;
    int groundN = 0;
    float headSum = 0.f;
    int headN = 0;
    float skateSum = 0.f;
    int skateN = 0;
    int samplesAll = 0;
    for (const NamedClip& nc : clips) {
      if (nc.clip == nullptr) {
        continue;
      }
      const bool standing = Contains(nc.name, "ready");
      const bool moving = Contains(nc.name, "walk") || Contains(nc.name, "jog") ||
                          Contains(nc.name, "run") || Contains(nc.name, "strafe");
      if (!standing && !moving) {
        continue;
      }
      const uint32_t frames = FrameCount(*nc.clip);
      float minFoot = 1e9f;
      float headZ = 0.f;
      float footSlide = 0.f;
      float bodyTravel = 0.f;
      WorldPose prev;
      for (uint32_t f = 0; f < frames; ++f) {
        const WorldPose pose =
            EchoesPose(echoes, *nc.clip, float(f) * nc.clip->interval, conventions[ci]);
        ++samplesAll;
        for (int k = 0; k < 2; ++k) {
          for (int bone : {balls[k], ankles[k]}) {
            if (bone >= 0) {
              minFoot = std::min(minFoot, pose.positions[size_t(bone)].z);
            }
          }
          if (ankles[k] >= 0 && knees[k] >= 0 &&
              pose.positions[size_t(ankles[k])].z > pose.positions[size_t(knees[k])].z + 0.05f) {
            ++sc.broken;
          }
        }
        if (head >= 0 && hips >= 0 &&
            pose.positions[size_t(head)].z < pose.positions[size_t(hips)].z) {
          ++sc.broken;
        }
        if (head >= 0) {
          headZ += pose.positions[size_t(head)].z;
        }
        if (moving && f > 0 && balls[0] >= 0 && balls[1] >= 0 && hips >= 0) {
          // The foot lower at both ends of the step is the planted one.
          const size_t lb = size_t(balls[0]);
          const size_t rb = size_t(balls[1]);
          const bool leftDown = prev.positions[lb].z <= prev.positions[rb].z;
          const size_t planted = leftDown ? lb : rb;
          const size_t other = leftDown ? rb : lb;
          if (pose.positions[planted].z <= pose.positions[other].z) {
            footSlide += Horizontal(Sub(pose.positions[planted], prev.positions[planted]));
            bodyTravel +=
                Horizontal(Sub(pose.positions[size_t(hips)], prev.positions[size_t(hips)]));
          }
        }
        prev = pose;
      }
      if (minFoot < 1e8f) {
        groundSum += std::fabs(minFoot);
        ++groundN;
      }
      if (standing && head >= 0) {
        headSum += std::fabs(headZ / float(frames) - bindHead);
        ++headN;
      }
      if (moving && bodyTravel > 0.05f) {
        skateSum += footSlide / bodyTravel;
        ++skateN;
      }
    }
    sc.ground = groundN != 0 ? groundSum / float(groundN) : 0.f;
    sc.head = headN != 0 ? headSum / float(headN) : 0.f;
    sc.skate = skateN != 0 ? skateSum / float(skateN) : 0.f;
    const float brokenShare = samplesAll != 0 ? float(sc.broken) / float(samplesAll) : 0.f;
    sc.score = sc.ground * 4.f + sc.head * 2.f + sc.skate + brokenShare * 2.f;
    scores.push_back(sc);
  }
  std::stable_sort(
      scores.begin(), scores.end(),
      [](const ConventionScore& a, const ConventionScore& b) { return a.score < b.score; });
  return scores;
}

std::string BuildAnimationCheck(const Skeleton& echoes, const Skeleton& prime,
                                const std::vector< ClipInput >& clips) {
  std::string out;
  Line(out, "== Animation decode check ==");
  float asStored = 0.f;
  float asInverse = 0.f;
  SelfCheck(echoes, asStored, asInverse);
  Line(out,
       "Skeleton self-check: local = parent^-1 * bind off by %.2f deg at most; bind * parent^-1 "
       "off by %.2f deg",
       asStored, asInverse);
  Line(out, "Bind rotations read as %s",
       BindStoredInverse(echoes) ? "stored inverted (world-to-bone)" : "stored as they are");

  std::vector< Clip > decoded(clips.size());
  std::vector< bool > ok(clips.size(), false);
  for (size_t ci = 0; ci < clips.size(); ++ci) {
    const ClipInput& in = clips[ci];
    Line(out, "");
    if (in.data.empty()) {
      Line(out, "%s: not found", in.name.c_str());
      continue;
    }
    Clip& clip = decoded[ci];
    std::string error;
    if (!ParseClip(in.data.data(), in.data.size(), clip, error)) {
      std::string head;
      char buf[4];
      for (size_t k = 0; k < in.data.size() && k < 8; ++k) {
        std::snprintf(buf, sizeof(buf), "%02X", in.data[k]);
        head += buf;
      }
      Line(out, "%s: could not decode (%s); %zu bytes, starting %s", in.name.c_str(), error.c_str(),
           in.data.size(), head.c_str());
      continue;
    }
    ok[ci] = true;
    size_t rot = 0;
    size_t trans = 0;
    size_t scale = 0;
    for (const Channel& c : clip.channels) {
      rot += !c.rotations.empty();
      trans += !c.translations.empty();
      scale += !c.scales.empty();
    }
    Line(out,
         "%s: version %u, %.3f s at %.4f s a frame, %s, %zu keys, %zu channels (%zu rotate, %zu "
         "move, %zu scale)",
         in.name.c_str(), clip.version, clip.duration, clip.interval,
         clip.looping ? "looping" : "once", clip.keyFrames.size(), clip.channels.size(), rot, trans,
         scale);
    if (clip.version == 2) {
      Line(out,
           "  bitstream: %llu of %llu bits used; %u keys after frame 0, channels count up to %u; "
           "table: %u word(s) "
           "after the bitmap, %u-byte bone ids%s",
           static_cast< unsigned long long >(clip.bitsUsed),
           static_cast< unsigned long long >(clip.bitsAvailable), clip.keyBitmapKeys,
           clip.descriptorKeys, clip.wordsAfterBitmap, clip.boneIdBytes,
           clip.layoutTies != 0 ? " (another layout fit as well)" : "");
    }
  }

  // The first standing clip's first frame against the bind pose: which reading of a key gives a
  // standing body back is told by which of these the keys sit near.
  const BindFrames bind = Binds(echoes);
  const auto index = IndexById(echoes);
  for (size_t ci = 0; ci < clips.size(); ++ci) {
    if (!ok[ci] || !Contains(clips[ci].name, "ready")) {
      continue;
    }
    Line(out, "");
    Line(out, "Keys of %s at frame 0 against the bind pose (angles in degrees; translations in m):",
         clips[ci].name.c_str());
    Line(out, "  %-20s %8s %8s %10s %8s", "bone", "identity", "bindloc", "bindloc^-1", "bindworld");
    for (const Channel& ch : decoded[ci].channels) {
      const auto bi = index.find(ch.bone);
      if (bi == index.end()) {
        continue;
      }
      const size_t i = bi->second;
      const Bone& b = echoes.bones[i];
      if (!ch.rotations.empty()) {
        const Quat k = ch.rotations.front();
        Line(out, "  %-20s %8.1f %8.1f %10.1f %8.1f", b.name.c_str(), AngleBetween(k, Quat{}),
             AngleBetween(k, bind.local[i]), AngleBetween(k, Conj(bind.local[i])),
             AngleBetween(k, bind.world[i]));
      }
      if (!ch.translations.empty()) {
        const Vec3 t = ch.translations.front();
        const auto p = index.find(b.parent);
        const bool hasParent = p != index.end() && p->second != i;
        const Vec3 d = hasParent ? Sub(ToVec(b.position), ToVec(echoes.bones[p->second].position))
                                 : ToVec(b.position);
        const Quat pw = hasParent ? bind.world[p->second] : Quat{};
        const Vec3 boundD = Rotate(Conj(pw), d);
        Line(out,
             "  %-20s t (%6.3f %6.3f %6.3f)  bind offset unrotated (%6.3f %6.3f %6.3f)  in "
             "parent's bind frame "
             "(%6.3f %6.3f %6.3f)",
             b.name.c_str(), t.x, t.y, t.z, d.x, d.y, d.z, boundD.x, boundD.y, boundD.z);
      }
    }
    break;
  }

  std::vector< NamedClip > named;
  for (size_t ci = 0; ci < clips.size(); ++ci) {
    if (ok[ci]) {
      named.push_back({clips[ci].name, &decoded[ci]});
    }
  }
  const std::vector< ConventionScore > ranking = RankConventions(echoes, named);
  const auto& conventions = Conventions();
  Line(out, "");
  Line(out, "How the keys apply, ranked (feet on the floor, head at its height standing, planted "
            "feet not sliding):");
  Line(out, "  %3s %-36s %7s %7s %7s %7s %6s", "#", "convention", "score", "floor", "head", "slide",
       "folded");
  for (const ConventionScore& sc : ranking) {
    Line(out, "  %3zu %-36s %7.3f %7.3f %7.3f %7.3f %6d", sc.index,
         ConventionName(conventions[sc.index]).c_str(), sc.score, sc.ground, sc.head, sc.skate,
         sc.broken);
  }

  // The two best as joint positions, for a look at the figure itself.
  static const char* const kJoints[] = {
      "Skeleton_Root", "Spine_2", "Collar",  "Head_1",     "L_hip",   "L_knee",
      "L_ankle",       "L_ball",  "R_hip",   "R_knee",     "R_ankle", "R_ball",
      "L_shoulder",    "L_elbow", "L_wrist", "R_shoulder", "R_elbow", "GUN_LCTR"};
  for (size_t rank = 0; rank < 2 && rank < ranking.size(); ++rank) {
    const Convention& c = conventions[ranking[rank].index];
    for (size_t ci = 0; ci < clips.size(); ++ci) {
      if (!ok[ci] || !(Contains(clips[ci].name, "ready") || clips[ci].name == "B_run_samus" ||
                       clips[ci].name == "B_strafe_Left_samus")) {
        continue;
      }
      const Clip& clip = decoded[ci];
      for (int q = 0; q < 4; ++q) {
        const float t = clip.duration * float(q) / 4.f;
        const WorldPose pose = EchoesPose(echoes, clip, t, c);
        std::string joints;
        for (const char* name : kJoints) {
          const int j = FindBone(echoes, name);
          if (j < 0) {
            continue;
          }
          char item[64];
          const Vec3& v = pose.positions[size_t(j)];
          std::snprintf(item, sizeof(item), " %s %.2f %.2f %.2f", name, v.x, v.y, v.z);
          joints += item;
        }
        Line(out, "  pose #%zu %s t=%.3f:%s", ranking[rank].index, clips[ci].name.c_str(), t,
             joints.c_str());
        if (Contains(clips[ci].name, "ready")) {
          break;
        }
      }
    }
  }

  // The best one carried onto Prime's skeleton: shared joints should land where they are on
  // Echoes'.
  if (!ranking.empty()) {
    float worst = 0.f;
    const Convention& c = conventions[ranking.front().index];
    for (size_t ci = 0; ci < clips.size(); ++ci) {
      if (!ok[ci]) {
        continue;
      }
      for (int q = 0; q < 8; ++q) {
        const WorldPose pose =
            EchoesPose(echoes, decoded[ci], decoded[ci].duration * float(q) / 8.f, c);
        const PrimePose onPrime = RetargetToPrime(echoes, pose, prime);
        for (size_t i = 0; i < prime.bones.size(); ++i) {
          const int j = FindBone(echoes, prime.bones[i].name.c_str());
          if (j >= 0) {
            worst = std::max(worst, Length(Sub(onPrime.positions[i], pose.positions[size_t(j)])));
          }
        }
      }
    }
    Line(out, "On Prime's skeleton, with #%zu: shared joints off by %.4f m at most",
         ranking.front().index, worst);
  }
  return out;
}

} // namespace PortEchoesAnim
