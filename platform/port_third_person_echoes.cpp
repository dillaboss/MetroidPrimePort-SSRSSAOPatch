// Echoes' Samus animations for the third-person body (see port_third_person_echoes.h).
#include "port_third_person_echoes.h"

#include "port_echoes_locomotion.h"
#include "port_echoes_probe.h"
#include "port_log.h"

#include "Kyoto/Animation/CCharLayoutInfo.hpp"
#include "Kyoto/Math/CQuaternion.hpp"
#include "Kyoto/Math/CVector3f.hpp"
#include "MetroidPrime/CAnimData.hpp"
#include "MetroidPrime/CStateManager.hpp"
#include "MetroidPrime/Player/CPlayer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

namespace PortThirdPersonEchoes {
namespace {

// The read of the image, shared with the thread that does it. Never destroyed: the thread may
// still hold it at exit.
struct Shared {
  std::mutex mutex;
  bool enabled = false;
  std::string imagePath;
  std::string loadedPath; // the image the read below is of
  bool loading = false;
  std::string error;
  std::shared_ptr< const PortEchoesLocomotion::Library > library;
  bool playing = false;
};
Shared& S() {
  static Shared* shared = new Shared;
  return *shared;
}

// The game-thread side: the blend, and the body it is mapped onto.
PortEchoesLocomotion::Player sPlayer;
std::shared_ptr< const PortEchoesLocomotion::Library > sLibrary;
const CAnimData* sBody = nullptr;
std::vector< int > sSegOf; // segment id of each bone of the skeleton handed to the player
bool sMapped = false;      // the body shares bones with Echoes' Samus
bool sPosed = false;       // the last Tick posed
bool sWasGrounded = true;
float sPrevRise = 0.f;
float sLastYaw = 0.f;
bool sHaveYaw = false;
int sPendingShot = -1;
std::vector< PortEchoesAnim::Quat > sDeltas;

void SetPlaying(bool playing) {
  std::lock_guard lock(S().mutex);
  S().playing = playing;
}

void StartRead(const std::string& path) {
  PortLog::Write("third person: reading Echoes' animations from %s\n", path.c_str());
  std::thread([path] {
    PortEchoesProbe::Skeleton skeleton;
    std::vector< std::pair< std::string, std::vector< uint8_t > > > clips;
    std::string error;
    std::shared_ptr< PortEchoesLocomotion::Library > library;
    if (PortEchoesProbe::LoadEchoesSamus(path, PortEchoesLocomotion::ClipNames(), skeleton, clips,
                                         error)) {
      auto built = std::make_shared< PortEchoesLocomotion::Library >();
      if (PortEchoesLocomotion::BuildLibrary(skeleton, clips, *built, error)) {
        library = std::move(built);
      }
    }
    Shared& shared = S();
    std::lock_guard lock(shared.mutex);
    if (shared.loadedPath != path) {
      return; // another image was chosen meanwhile
    }
    shared.loading = false;
    shared.library = library;
    shared.error = library != nullptr ? std::string() : error;
    if (library != nullptr) {
      std::string missing;
      for (const std::string& name : library->missing) {
        missing += " " + name;
      }
      PortLog::Write("third person: Echoes' animations ready (%zu; not on the image:%s)\n",
                     library->clips.size(), missing.empty() ? " none" : missing.c_str());
    } else {
      PortLog::Write("third person: Echoes' animations not read: %s\n", error.c_str());
    }
  }).detach();
}

// Prime's skeleton as the body's layout has it, by name, with each bone's segment id.
PortEchoesProbe::Skeleton BodySkeleton(const CCharLayoutInfo& layout, std::vector< int >& segOf) {
  PortEchoesProbe::Skeleton skeleton;
  segOf.clear();
  for (auto it = layout.PortNameMap().begin(); it != layout.PortNameMap().end(); ++it) {
    const CSegId seg = it->second;
    if (seg == CSegId::Invalid() || seg == CSegId::Null() || !layout.ContainsSegment(seg)) {
      continue;
    }
    PortEchoesProbe::Bone bone;
    bone.id = seg.val();
    bone.parent = layout.GetOriginalParent(seg).val();
    const CVector3f p = layout.GetReferenceStanceOffset(seg);
    bone.position[0] = p.GetX();
    bone.position[1] = p.GetY();
    bone.position[2] = p.GetZ();
    bone.name.assign(it->first.data(), it->first.size());
    skeleton.bones.push_back(bone);
    segOf.push_back(seg.val());
  }
  return skeleton;
}

} // namespace

bool Enabled() {
  std::lock_guard lock(S().mutex);
  return S().enabled;
}

void SetEnabled(bool on) {
  std::lock_guard lock(S().mutex);
  S().enabled = on;
}

std::string ImagePath() {
  std::lock_guard lock(S().mutex);
  return S().imagePath;
}

void SetImagePath(const std::string& path) {
  std::lock_guard lock(S().mutex);
  S().imagePath = path;
}

void Load() {
  Shared& shared = S();
  std::string path;
  {
    std::lock_guard lock(shared.mutex);
    if (!shared.enabled || shared.imagePath.empty() || shared.loadedPath == shared.imagePath) {
      return;
    }
    path = shared.imagePath;
    shared.loadedPath = path;
    shared.loading = true;
    shared.error.clear();
    shared.library.reset();
  }
  StartRead(path);
}

State GetState() {
  Shared& shared = S();
  std::lock_guard lock(shared.mutex);
  State state;
  if (!shared.enabled || shared.imagePath.empty() || shared.loadedPath != shared.imagePath) {
    return state;
  }
  if (shared.loading) {
    state.phase = State::Phase::Loading;
  } else if (shared.library != nullptr) {
    state.phase = State::Phase::Ready;
    state.clips = shared.library->clips.size();
    state.missing = shared.library->missing;
    state.playing = shared.playing;
  } else {
    state.phase = State::Phase::Failed;
    state.message = shared.error;
  }
  return state;
}

bool Tick(const CStateManager& mgr, const CAnimData& body, float dt, float fwd, float side,
          bool grounded, float rise, float scale) {
  Load();
  std::shared_ptr< const PortEchoesLocomotion::Library > library;
  {
    std::lock_guard lock(S().mutex);
    if (S().enabled && S().loadedPath == S().imagePath) {
      library = S().library;
    }
  }
  if (library == nullptr) {
    if (sPosed) {
      SetPlaying(false);
    }
    sPosed = false;
    return false;
  }
  if (library != sLibrary) {
    sLibrary = library;
    sPlayer.SetLibrary(library);
  }
  if (sBody != &body) {
    sBody = &body;
    const CCharLayoutInfo* layout = body.GetCharLayoutInfo();
    sMapped = false;
    if (layout != nullptr) {
      const PortEchoesProbe::Skeleton prime = BodySkeleton(*layout, sSegOf);
      int shared = 0;
      for (const PortEchoesProbe::Bone& b : prime.bones) {
        shared += library->echoes.Find(b.name) != nullptr;
      }
      sMapped = shared >= 8;
      sPlayer.SetPrime(prime);
      PortLog::Write("third person: Echoes' animations drive %d of the body's %zu bones\n", shared,
                     prime.bones.size());
    }
  }
  if (!sMapped) {
    return sPosed = false;
  }
  if (dt <= 0.f) {
    return sPosed; // a tick that doesn't advance keeps the last pose
  }

  const CPlayer* player = mgr.GetPlayer();
  PortEchoesLocomotion::Input in;
  in.dt = dt;
  in.forwardSpeed = fwd;
  in.sideSpeed = side;
  in.onGround = grounded;
  in.scale = std::max(scale, 0.01f);
  in.aim = false;        // the body's own aim clips and arm turn aim
  in.layerShots = false; // over the finished pose (RecoilDeltas)
  // A take-off leaves the ground going up; a double jump kicks the climb back up in the air.
  in.jumped = sWasGrounded && !grounded && rise > 1.f;
  in.doubleJumped = !sWasGrounded && !grounded && rise - sPrevRise > 3.f;
  sWasGrounded = grounded;
  sPrevRise = rise;
  if (player != nullptr) {
    in.dashing = player->IsSidewaysDashing();
    const CVector3f facing = player->GetTransform().GetForward();
    const float yaw = std::atan2(facing.GetX(), facing.GetY());
    // atan2(x, y) grows turning right (clockwise from above); the turn rate is left positive.
    in.turnRate = sHaveYaw ? -std::remainder(yaw - sLastYaw, 2.f * 3.14159265f) / dt : 0.f;
    sLastYaw = yaw;
    sHaveYaw = true;
  }
  if (sPendingShot >= 0) {
    sPlayer.Shoot(sPendingShot);
    sPendingShot = -1;
  }
  sPlayer.Update(in);
  if (!sPosed) {
    SetPlaying(true);
  }
  sPosed = true;
  return true;
}

bool BasePose(const CAnimData& body, CQuaternion* rot, CVector3f* off, int count) {
  if (!sPosed || sBody != &body) {
    return false;
  }
  const CCharLayoutInfo* layout = body.GetCharLayoutInfo();
  if (layout == nullptr) {
    return false;
  }
  for (const CSegId& seg : layout->GetBodyPartSegIds()) {
    if (seg.val() < count) {
      rot[seg.val()] = CQuaternion::NoRotation();
      off[seg.val()] = layout->GetFromParentUnrotated(seg);
    }
  }
  const PortEchoesAnim::PrimePose& p = sPlayer.Pose();
  for (size_t i = 0; i < p.locals.size() && i < sSegOf.size(); ++i) {
    const int seg = sSegOf[i];
    // The root (3) is the actor's: the pose builder never takes it from a clip either.
    if (seg >= count || seg == 3) {
      continue;
    }
    const PortEchoesAnim::Quat& q = p.locals[i];
    rot[seg] = CQuaternion(q.w, q.x, q.y, q.z);
    if (p.hasOffset[i]) {
      const PortEchoesAnim::Vec3& o = p.offsets[i];
      off[seg] = CVector3f(o.x, o.y, o.z);
    }
  }
  return true;
}

bool RecoilDeltas(const CAnimData& body, CQuaternion* delta, int count) {
  if (!sPosed || sBody != &body || !sPlayer.ShotDeltas(sDeltas)) {
    return false;
  }
  for (int i = 0; i < count; ++i) {
    delta[i] = CQuaternion::NoRotation();
  }
  for (size_t i = 0; i < sDeltas.size() && i < sSegOf.size(); ++i) {
    if (sSegOf[i] < count) {
      const PortEchoesAnim::Quat& q = sDeltas[i];
      delta[sSegOf[i]] = CQuaternion(q.w, q.x, q.y, q.z);
    }
  }
  return true;
}

void NoteShot(int strength) { sPendingShot = std::max(sPendingShot, strength); }

void Reset() {
  sBody = nullptr;
  sMapped = false;
  sPosed = false;
  sWasGrounded = true;
  sPrevRise = 0.f;
  sHaveYaw = false;
  sPendingShot = -1;
  sPlayer.Reset();
  SetPlaying(false);
}

} // namespace PortThirdPersonEchoes
