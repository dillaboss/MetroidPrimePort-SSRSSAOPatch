// Skeleton and character-set parsing and the comparison report (see port_echoes_probe.h). Pure:
// bytes in, text out. The disc reading and the background run are port_echoes_probe_import.cpp.
//
// Layouts (big-endian, strings NUL-terminated), from Prime's own readers (CCharacterInfo,
// CPASDatabase, CEffectComponent) and the Retro modding wiki for Echoes' additions:
//  CINF: u32 bone count; per bone: u32 id, u32 parent, f32[3] position, Echoes only: f32[4]
//        rotation and f32[4] local rotation (w, x, y, z), u32 link count, u32[] links; u32 build
//        order count, u32[]; u32 name count; per name: string, u32 id.
//  ANCS: u16 version, u16 character set version, u32 count; per character: u32 id, u16 version,
//        string name, u32 model, skin, skeleton, u32 animation name count, per name: u32 id,
//        (version < 10: string, always empty), string name; PAS4 database; particle lists;
//        u32; (10+: u32); (2+: AABBs); (3+: effects); (4+: frozen model, skin); (5+: animation
//        ids); (10+: u32 spatial primitives, u8, indexed AABBs).
#include "port_echoes_probe.h"

#include "port_echoes_anim.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>

namespace PortEchoesProbe {
namespace {

class Reader {
public:
  Reader(const uint8_t* data, size_t size) : mData(data), mSize(size) {}
  bool ok() const { return mOk; }
  size_t pos() const { return mPos; }
  bool Has(size_t n) const { return mOk && mSize - mPos >= n; }
  uint8_t U8() {
    if (!Has(1)) {
      return Fail(), 0;
    }
    return mData[mPos++];
  }
  uint16_t U16() {
    if (!Has(2)) {
      return Fail(), 0;
    }
    const uint16_t v = uint16_t((mData[mPos] << 8) | mData[mPos + 1]);
    mPos += 2;
    return v;
  }
  uint32_t U32() {
    if (!Has(4)) {
      return Fail(), 0;
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
      if (!Has(1)) {
        Fail();
        return s;
      }
      const char c = char(mData[mPos++]);
      if (c == '\0') {
        return s;
      }
      // No name in these files is near this long; a longer one is a misread.
      if (s.size() >= 1024) {
        Fail();
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
  // A count that must fit in what is left, each element at least `minBytes` long.
  uint32_t Count(size_t minBytes, uint32_t limit = 100000) {
    const uint32_t n = U32();
    if (!mOk || n > limit || (minBytes != 0 && uint64_t(n) * minBytes > mSize - mPos)) {
      Fail();
      return 0;
    }
    return n;
  }
  void Fail() { mOk = false; }

private:
  const uint8_t* mData;
  size_t mSize;
  size_t mPos = 0;
  bool mOk = true;
};

std::string Lower(std::string s) {
  for (char& c : s) {
    c = char(std::tolower(static_cast< unsigned char >(c)));
  }
  return s;
}

std::string Hex(uint32_t v) {
  char buf[12];
  std::snprintf(buf, sizeof(buf), "%08X", v);
  return buf;
}

// The PAS4 animation database: only skipped (its states pick animations for AI bodies).
bool SkipPas(Reader& in, std::string& error) {
  const uint32_t magic = in.U32();
  if (!in.ok() || magic != 0x50415334) { // 'PAS4'
    error = "no PAS4 database where one was expected";
    return false;
  }
  const uint32_t states = in.Count(12, 1000);
  in.U32(); // default state
  for (uint32_t s = 0; s < states && in.ok(); ++s) {
    in.U32(); // state id
    const uint32_t parmCount = in.Count(0, 64);
    const uint32_t animCount = in.Count(4, 10000);
    std::vector< uint32_t > types(parmCount);
    for (uint32_t p = 0; p < parmCount && in.ok(); ++p) {
      types[p] = in.U32();
      in.U32(); // weight function
      in.F32(); // weight
      switch (types[p]) {
      case 0: // int
      case 1: // uint
      case 2: // float
      case 4: // enum
        in.Skip(8);
        break;
      case 3: // bool
        in.Skip(2);
        break;
      default:
        break;
      }
    }
    for (uint32_t a = 0; a < animCount && in.ok(); ++a) {
      in.U32(); // animation id
      for (uint32_t p = 0; p < parmCount && in.ok(); ++p) {
        in.Skip(types[p] == 3 ? 1 : types[p] <= 4 ? 4 : 0);
      }
    }
  }
  if (!in.ok()) {
    error = "the PAS4 database runs past the end";
    return false;
  }
  return true;
}

// A u32 count and that many u32 ids.
void SkipIdList(Reader& in) {
  const uint32_t n = in.Count(4);
  in.Skip(size_t(n) * 4);
}

bool ParseCharacter(Reader& in, Character& c, std::string& error) {
  c.id = in.U32();
  c.version = in.U16();
  c.name = in.String();
  c.model = in.U32();
  c.skin = in.U32();
  c.skeleton = in.U32();
  const uint32_t names = in.Count(5, 5000);
  for (uint32_t i = 0; i < names && in.ok(); ++i) {
    const uint32_t id = in.U32();
    if (c.version < 10) {
      in.String(); // always empty
    }
    c.animations.emplace_back(id, in.String());
  }
  if (!in.ok()) {
    error = "character '" + c.name + "' is cut short in its animation names";
    return false;
  }
  if (!SkipPas(in, error)) {
    error = "character '" + c.name + "': " + error;
    return false;
  }
  // Particles: generic, swoosh, electric; one more list from version 6, another (spawn) in 10.
  SkipIdList(in);
  SkipIdList(in);
  SkipIdList(in);
  if (c.version > 5) {
    SkipIdList(in);
  }
  if (c.version >= 10) {
    SkipIdList(in);
  }
  in.U32();
  if (c.version >= 10) {
    in.U32();
  }
  if (c.version > 1) {
    const uint32_t boxes = in.Count(25, 10000);
    for (uint32_t i = 0; i < boxes && in.ok(); ++i) {
      in.String();
      in.Skip(24);
    }
  }
  if (c.version > 2) {
    const uint32_t effects = in.Count(5, 10000);
    for (uint32_t i = 0; i < effects && in.ok(); ++i) {
      in.String();
      const uint32_t parts = in.Count(22, 1000);
      for (uint32_t p = 0; p < parts && in.ok(); ++p) {
        in.String(); // component name
        in.Skip(8);  // particle type and id
        in.String(); // bone name
        in.Skip(12); // scale, parented mode, flags
      }
    }
  }
  if (c.version > 3) {
    in.Skip(8); // frozen model, skin
  }
  if (c.version > 4) {
    SkipIdList(in);
  }
  if (c.version >= 10) {
    in.U32(); // spatial primitives
    in.U8();
    const uint32_t boxes = in.Count(28, 10000);
    in.Skip(size_t(boxes) * 28);
  }
  if (!in.ok()) {
    error =
        "character '" + c.name + "' (version " + std::to_string(c.version) + ") runs past the end";
    return false;
  }
  return true;
}

float RotationAngle(const float q[4]) {
  const float w = std::min(std::fabs(q[0]), 1.f);
  return 2.f * std::acos(w) * 57.29578f;
}

float Distance(const float a[3], const float b[3]) {
  const float dx = a[0] - b[0];
  const float dy = a[1] - b[1];
  const float dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string ParentName(const Skeleton& s, const Bone& b) {
  const Bone* p = s.Find(b.parent);
  return p != nullptr ? p->name
                      : (b.parent == 0 || b.parent == 0xFFFFFFFFu ? "(root)"
                                                                  : "#" + std::to_string(b.parent));
}

void Line(std::string& out, const char* format, ...) {
  char buf[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  out += buf;
  out += '\n';
}

// The candidate the comparison uses: Prime's player body is named "Samus"; Echoes' most complete
// Samus is the one with the most animations.
const Found* Pick(const DiscScan& scan, bool byAnimations) {
  const Found* best = nullptr;
  for (const Found& f : scan.samus) {
    if (!f.skeletonError.empty()) {
      continue;
    }
    if (best == nullptr) {
      best = &f;
      continue;
    }
    if (byAnimations) {
      if (f.character.animations.size() > best->character.animations.size()) {
        best = &f;
      }
    } else {
      const bool exact = Lower(f.character.name) == "samus";
      const bool bestExact = Lower(best->character.name) == "samus";
      if ((exact && !bestExact) ||
          (exact == bestExact && f.skeleton.bones.size() > best->skeleton.bones.size())) {
        best = &f;
      }
    }
  }
  return best;
}

void ListCandidates(std::string& out, const DiscScan& scan) {
  for (const Found& f : scan.samus) {
    Line(out, "  %-28s ANCS %s in %-18s v%u  skeleton %s: %s, %zu animations",
         f.character.name.c_str(), Hex(f.ancs).c_str(), f.pak.c_str(),
         unsigned(f.character.version), Hex(f.character.skeleton).c_str(),
         f.skeletonError.empty() ? (std::to_string(f.skeleton.bones.size()) + " bones").c_str()
                                 : f.skeletonError.c_str(),
         f.character.animations.size());
  }
}

void ListBones(std::string& out, const Skeleton& s, bool rotations) {
  for (const Bone& b : s.bones) {
    if (rotations) {
      Line(out, "  %3u %-24s parent %-20s pos (%7.3f %7.3f %7.3f)  rot %5.1f deg  local %5.1f deg",
           b.id, b.name.c_str(), ParentName(s, b).c_str(), b.position[0], b.position[1],
           b.position[2], RotationAngle(b.rotation), RotationAngle(b.localRotation));
    } else {
      Line(out, "  %3u %-24s parent %-20s pos (%7.3f %7.3f %7.3f)", b.id, b.name.c_str(),
           ParentName(s, b).c_str(), b.position[0], b.position[1], b.position[2]);
    }
  }
}

} // namespace

const Found* PickSamus(const DiscScan& scan, Game game) { return Pick(scan, game == Game::Echoes); }

const Bone* Skeleton::Find(uint32_t id) const {
  for (const Bone& b : bones) {
    if (b.id == id) {
      return &b;
    }
  }
  return nullptr;
}

const Bone* Skeleton::Find(const std::string& name) const {
  const std::string want = Lower(name);
  for (const Bone& b : bones) {
    if (Lower(b.name) == want) {
      return &b;
    }
  }
  return nullptr;
}

bool ParseSkeleton(const uint8_t* data, size_t size, Game game, Skeleton& out, std::string& error) {
  out = {};
  Reader in(data, size);
  const uint32_t count = in.Count(24, 1000);
  for (uint32_t i = 0; i < count && in.ok(); ++i) {
    Bone b;
    b.id = in.U32();
    b.parent = in.U32();
    for (float& v : b.position) {
      v = in.F32();
    }
    if (game == Game::Echoes) {
      for (float& v : b.rotation) {
        v = in.F32();
      }
      for (float& v : b.localRotation) {
        v = in.F32();
      }
    }
    SkipIdList(in);
    out.bones.push_back(b);
  }
  SkipIdList(in); // build order
  const uint32_t names = in.Count(5, 1000);
  for (uint32_t i = 0; i < names && in.ok(); ++i) {
    const std::string name = in.String();
    const uint32_t id = in.U32();
    for (Bone& b : out.bones) {
      if (b.id == id) {
        b.name = name;
      }
    }
  }
  if (!in.ok() || out.bones.empty()) {
    error = "not a " + std::string(game == Game::Echoes ? "Echoes" : "Prime") + " skeleton (CINF)";
    out = {};
    return false;
  }
  for (Bone& b : out.bones) {
    if (b.name.empty()) {
      b.name = "#" + std::to_string(b.id);
    }
  }
  return true;
}

bool ParseCharacterSet(const uint8_t* data, size_t size, std::vector< Character >& out,
                       std::string& error, size_t* end) {
  out.clear();
  if (end != nullptr) {
    *end = 0;
  }
  Reader in(data, size);
  const uint16_t fileVersion = in.U16();
  const uint16_t setVersion = in.U16();
  const uint32_t count = in.Count(20, 1000);
  if (!in.ok() || fileVersion != 1 || setVersion != 1) {
    error = "not a character set (ANCS)";
    return false;
  }
  for (uint32_t i = 0; i < count; ++i) {
    Character c;
    std::string why;
    const bool whole = ParseCharacter(in, c, why);
    // A character whose animation names were read is kept even if the rest of it was not.
    if (!c.name.empty() && (whole || !c.animations.empty())) {
      out.push_back(std::move(c));
    }
    if (!whole) {
      error = why + " (character " + std::to_string(i + 1) + " of " + std::to_string(count) + ")";
      return !out.empty();
    }
  }
  if (end != nullptr) {
    *end = in.pos();
  }
  return true;
}

bool IsSamusCharacter(const std::string& name) {
  const std::string n = Lower(name);
  return n.find("samus") != std::string::npos && n.find("gun") == std::string::npos &&
         n.find("beam") == std::string::npos;
}

std::string BuildReport(const DiscScan& prime, const DiscScan& echoes) {
  std::string out;
  Line(out, "Metroid Prime / Metroid Prime 2: Echoes - Samus skeleton and animation report");
  Line(out, "Names and numbers only: no game data is in this file.");
  Line(out, "");
  Line(out, "Prime disc:  %s",
       prime.id.empty() ? ("not read: " + prime.error).c_str() : prime.id.c_str());
  Line(out, "Echoes disc: %s",
       echoes.id.empty() ? ("not read: " + echoes.error).c_str() : echoes.id.c_str());
  if (!prime.id.empty() && !prime.error.empty()) {
    Line(out, "Prime: %s", prime.error.c_str());
  }
  if (!echoes.id.empty() && !echoes.error.empty()) {
    Line(out, "Echoes: %s", echoes.error.c_str());
  }
  Line(out, "");
  Line(out, "== Samus characters found ==");
  Line(out, "Prime (%d character sets read):", prime.characterSets);
  ListCandidates(out, prime);
  Line(out, "Echoes (%d character sets read):", echoes.characterSets);
  ListCandidates(out, echoes);

  const Found* p = Pick(prime, false);
  const Found* e = Pick(echoes, true);
  Line(out, "");
  Line(out, "== Comparison ==");
  if (p == nullptr || e == nullptr) {
    Line(out, "Not possible: %s has no readable Samus skeleton.",
         p == nullptr ? "Prime" : "Echoes");
  } else {
    Line(out, "Prime  %s (%zu bones) vs Echoes %s (%zu bones, %zu animations)",
         p->character.name.c_str(), p->skeleton.bones.size(), e->character.name.c_str(),
         e->skeleton.bones.size(), e->character.animations.size());
    int same = 0;
    int sameParent = 0;
    int near = 0;
    int boundRotated = 0;
    std::vector< std::string > primeOnly;
    std::vector< std::string > echoesOnly;
    for (const Bone& b : p->skeleton.bones) {
      if (e->skeleton.Find(b.name) == nullptr) {
        primeOnly.push_back(b.name);
      }
    }
    for (const Bone& b : e->skeleton.bones) {
      if (RotationAngle(b.rotation) > 0.5f || RotationAngle(b.localRotation) > 0.5f) {
        ++boundRotated;
      }
      const Bone* q = p->skeleton.Find(b.name);
      if (q == nullptr) {
        echoesOnly.push_back(b.name);
        continue;
      }
      ++same;
      sameParent += Lower(ParentName(p->skeleton, *q)) == Lower(ParentName(e->skeleton, b));
      near += Distance(q->position, b.position) < 0.02f;
    }
    Line(out, "Bones with the same name:            %d (Prime has %zu, Echoes %zu)", same,
         p->skeleton.bones.size(), e->skeleton.bones.size());
    Line(out, "  ...with the same parent:           %d", sameParent);
    Line(out, "  ...within 2 cm of the same place:  %d", near);
    Line(out,
         "Echoes bones with a bind rotation:   %d (0 means Echoes' animations fit Prime's bones as "
         "they are)",
         boundRotated);
    std::string list;
    for (const std::string& n : primeOnly) {
      list += (list.empty() ? "" : ", ") + n;
    }
    Line(out, "Only in Prime:  %s", list.empty() ? "(none)" : list.c_str());
    list.clear();
    for (const std::string& n : echoesOnly) {
      list += (list.empty() ? "" : ", ") + n;
    }
    Line(out, "Only in Echoes: %s", list.empty() ? "(none)" : list.c_str());
    Line(out, "");
    Line(out, "Per bone (Echoes' bones; distance to Prime's bone of the same name):");
    for (const Bone& b : e->skeleton.bones) {
      const Bone* q = p->skeleton.Find(b.name);
      if (q == nullptr) {
        Line(out, "  %-24s (not in Prime)", b.name.c_str());
        continue;
      }
      Line(out, "  %-24s parent %s  moved %6.3f  rot %5.1f deg  local %5.1f deg", b.name.c_str(),
           Lower(ParentName(p->skeleton, *q)) == Lower(ParentName(e->skeleton, b)) ? "same    "
                                                                                   : "DIFFERS ",
           Distance(q->position, b.position), RotationAngle(b.rotation),
           RotationAngle(b.localRotation));
    }
  }

  if (p != nullptr && e != nullptr && (!echoes.clips.empty() || !echoes.clipError.empty())) {
    Line(out, "");
    if (!echoes.clipError.empty()) {
      Line(out, "== Animation decode check ==");
      Line(out, "Not possible: %s", echoes.clipError.c_str());
    } else {
      std::vector< PortEchoesAnim::ClipInput > clips;
      for (const auto& [name, data] : echoes.clips) {
        clips.push_back({name, data});
      }
      out += PortEchoesAnim::BuildAnimationCheck(e->skeleton, p->skeleton, clips);
    }
  }

  Line(out, "");
  Line(out, "== Echoes Samus animations ==");
  std::set< std::string > listed;
  for (const Found& f : echoes.samus) {
    std::string key = f.character.name + "/" + std::to_string(f.character.animations.size());
    if (!listed.insert(key).second) {
      continue;
    }
    Line(out, "%s (%zu):", f.character.name.c_str(), f.character.animations.size());
    for (const auto& [id, name] : f.character.animations) {
      Line(out, "  %4u %s", id, name.c_str());
    }
  }

  Line(out, "");
  Line(out, "== Skeletons ==");
  if (p != nullptr) {
    Line(out, "Prime %s:", p->character.name.c_str());
    ListBones(out, p->skeleton, false);
  }
  if (e != nullptr) {
    Line(out, "Echoes %s:", e->character.name.c_str());
    ListBones(out, e->skeleton, true);
  }

  if (!prime.notes.empty() || !echoes.notes.empty()) {
    Line(out, "");
    Line(out, "== Notes ==");
    for (const std::string& n : prime.notes) {
      Line(out, "Prime: %s", n.c_str());
    }
    for (const std::string& n : echoes.notes) {
      Line(out, "Echoes: %s", n.c_str());
    }
  }
  return out;
}

} // namespace PortEchoesProbe
