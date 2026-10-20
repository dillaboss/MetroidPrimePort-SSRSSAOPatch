// Experimental third-person view (see port_third_person.h).
#include "port_third_person.h"

#include "port_debug.h"
#include "port_echoes_locomotion.h"
#include "port_echoes_probe.h"
#include "port_freecam.h"
#include "port_log.h"

#include "Collision/CMaterialFilter.hpp"
#include "Collision/CMaterialList.hpp"
#include "Collision/CRayCastResult.hpp"
#include "Kyoto/Animation/CCharLayoutInfo.hpp"
#include "Kyoto/Animation/CHierarchyPoseBuilder.hpp"
#include "Kyoto/Math/CQuaternion.hpp"
#include "Kyoto/Math/CRelAngle.hpp"
#include "Kyoto/Math/CTransform4f.hpp"
#include "Kyoto/Math/CVector3f.hpp"
#include "MetroidPrime/CActor.hpp"
#include "MetroidPrime/CAnimData.hpp"
#include "MetroidPrime/CStateManager.hpp"
#include "MetroidPrime/Cameras/CCameraManager.hpp"
#include "MetroidPrime/Player/CPlayer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace PortThirdPerson {
namespace {

constexpr float kPi = 3.14159265f;
constexpr float kDeg = kPi / 180.f;

// What the camera and the crosshair ray stop at: solid world, not the player, characters,
// projectile-passthrough grates or surfaces made for cameras to pass.
const CMaterialFilter& LineOfSight() {
  static const CMaterialFilter filter = CMaterialFilter::MakeIncludeExclude(
      CMaterialList(kMT_Solid),
      CMaterialList(kMT_ProjectilePassthrough, kMT_Player, kMT_Character, kMT_CameraPassthrough));
  return filter;
}

// A rate-independent ease towards a target: the fraction of the gap closed in `dt` at `rate` per
// second (1 - e^-rate dt).
float Approach(float dt, float rate) { return 1.f - std::exp(-rate * dt); }

float Clamp(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

// Distance along a ray to the first solid surface, or `length` when there is none.
float RayDistance(const CStateManager& mgr, const CVector3f& from, const CVector3f& dir,
                  float length) {
  const CRayCastResult hit = mgr.RayStaticIntersection(from, dir, length, LineOfSight());
  if (!hit.IsValid()) {
    return length;
  }
  return std::min((hit.GetPoint() - from).Magnitude(), length);
}

// ---- The camera, per presented frame -----------------------------------------------------------

bool sActive = false;
bool sShowBody = false;
CTransform4f sView = CTransform4f::Identity();
float sWeight = 0.f;    // 0 first person, 1 fully behind; eases in when the camera appears
float sBoom = 0.f;      // how far back the camera sits along its arm, after walls
float sConverge = 20.f; // distance ahead at which the crosshair ray and the camera's centre meet
bool sFresh = true;     // the next active frame starts the smoothing over
std::chrono::steady_clock::time_point sLastFrame;

float FrameDt() {
  const auto now = std::chrono::steady_clock::now();
  float dt = std::chrono::duration< float >(now - sLastFrame).count();
  sLastFrame = now;
  // A long gap (a load, the pause menu, a breakpoint) restarts the eases instead of jumping.
  return std::isfinite(dt) ? Clamp(dt, 0.f, 0.1f) : 0.f;
}

// ---- The body, per simulation tick ------------------------------------------------------------

// The body's motion in its own frame (x right, y forward, z up): angles in radians, offsets in
// world units.
struct Motion {
  float roll = 0.f;  // about forward: positive tips the head to the right
  float pitch = 0.f; // about right: positive tips the head back
  float sway = 0.f;  // sideways
  float bob = 0.f;   // up
};

Motion sPrevMotion;
Motion sMotion;
uint sMotionGeneration = 0;
bool sMotionValid = false;

// Smoothed drivers of the motion.
float sLean = 0.f;
float sTip = 0.f;
float sStrideAmp = 0.f;
float sPhase = 0.f;
float sDip = 0.f; // landing squash, a damped spring
float sDipVel = 0.f;
float sPrevFallSpeed = 0.f;
bool sWasOnGround = true;
CVector3f sPrevForward = CVector3f(0.f, 1.f, 0.f);
bool sHavePrevForward = false;

// Spine bones bent towards the aim. Found by name when the body's skeleton is first seen; their
// rotation from the animation is kept apart from the bend, since the body's animation is frozen and
// re-reading the pose would return the last bend.
constexpr int kMaxSpine = 2;
const CAnimData* sSpineFor = nullptr;
int sSpineCount = 0;
CSegId sSpine[kMaxSpine] = {CSegId::Null(), CSegId::Null()};
CQuaternion sSpineBase[kMaxSpine] = {CQuaternion::NoRotation(), CQuaternion::NoRotation()};
CQuaternion sSpineSet[kMaxSpine] = {CQuaternion::NoRotation(), CQuaternion::NoRotation()};
bool sSpineBent = false;
float sBend = 0.f;

std::string Lower(const rstl::string& s) {
  std::string out(s.data(), s.size());
  for (char& c : out) {
    c = static_cast< char >(std::tolower(static_cast< unsigned char >(c)));
  }
  return out;
}

bool SameRotation(const CQuaternion& a, const CQuaternion& b) {
  return std::fabs(CQuaternion::Dot(a, b)) > 0.999999f;
}

// The spine bones of the body's skeleton, logged once so the names can be checked against a run.
void FindSpine(const CAnimData& anim) {
  sSpineFor = &anim;
  sSpineCount = 0;
  sSpineBent = false;
  const CCharLayoutInfo* layout = anim.GetCharLayoutInfo();
  if (layout == nullptr) {
    return;
  }
  std::string names;
  // Lower spine first. Retro's skeletons name sides with an l_/r_ prefix or a _l/_r suffix; those
  // are limbs, never the spine.
  static const char* const kWords[] = {"spine", "chest", "torso"};
  for (const char* word : kWords) {
    for (auto it = layout->PortNameMap().begin(); it != layout->PortNameMap().end(); ++it) {
      const std::string name = Lower(it->first);
      if (word == kWords[0]) {
        names += names.empty() ? "" : ", ";
        names.append(it->first.data(), it->first.size());
      }
      const bool sided = name.rfind("l_", 0) == 0 || name.rfind("r_", 0) == 0 ||
                         (name.size() > 2 && (name.compare(name.size() - 2, 2, "_l") == 0 ||
                                              name.compare(name.size() - 2, 2, "_r") == 0));
      if (sided || name.find(word) == std::string::npos || sSpineCount >= kMaxSpine) {
        continue;
      }
      bool seen = false;
      for (int i = 0; i < sSpineCount; ++i) {
        seen = seen || sSpine[i] == it->second;
      }
      if (!seen && it->second != CSegId::Invalid() && it->second != CSegId::Null()) {
        sSpine[sSpineCount++] = it->second;
      }
    }
  }
  PortLog::Write("third person: Samus's skeleton has bones: %s\n", names.c_str());
  PortLog::Write("third person: bending %d spine bone(s) towards the aim%s\n", sSpineCount,
                 sSpineCount == 0 ? " (none found by name: the torso stays as it is)" : "");
}

// Puts the spine back as the animation has it. A bone that no longer holds the bend has been given
// a new pose by the animation (a morph transition starting), which is left as it is.
void RestoreSpine(CAnimData& anim) {
  if (!sSpineBent || sSpineFor != &anim) {
    sSpineBent = false;
    return;
  }
  CHierarchyPoseBuilder& pose = anim.PoseBuilder();
  bool changed = false;
  for (int i = 0; i < sSpineCount; ++i) {
    if (SameRotation(pose.GetSegRotation(sSpine[i]), sSpineSet[i])) {
      pose.Insert(sSpine[i], sSpineBase[i]);
      changed = true;
    }
  }
  if (changed) {
    anim.SetPoseBuilderValid(false);
  }
  sSpineBent = false;
}

void BendSpine(CAnimData& anim, float bend) {
  if (sSpineFor != &anim) {
    FindSpine(anim);
  }
  if (sSpineCount == 0) {
    return;
  }
  // The pose tree as the animation has it, before reading it.
  anim.PreRender();
  CHierarchyPoseBuilder& pose = anim.PoseBuilder();
  // Retro's skeletons have no bind rotations, so a rotation about the parent's x is a bend about
  // the body's right axis: the share of each bone, applied after the animation's own rotation.
  const CQuaternion share =
      CQuaternion::XRotation(CRelAngle::FromRadians(bend / float(sSpineCount)));
  for (int i = 0; i < sSpineCount; ++i) {
    const CQuaternion now = pose.GetSegRotation(sSpine[i]);
    // Anything other than the last bend is the animation's (a new animation, a rebuilt tree).
    if (!sSpineBent || !SameRotation(now, sSpineSet[i])) {
      sSpineBase[i] = now;
    }
    sSpineSet[i] = share * sSpineBase[i];
    pose.Insert(sSpine[i], sSpineSet[i]);
  }
  anim.SetPoseBuilderValid(false);
  sSpineBent = true;
}

// Interpolated between the last two ticks, as everything else on screen is.
Motion PresentedMotion() {
  const float t = CCameraManager::GetPresentationInterpolation();
  if (t < 0.f || t >= 1.f || sMotionGeneration != CActor::PortTickGeneration()) {
    return sMotion;
  }
  Motion m;
  m.roll = sPrevMotion.roll + (sMotion.roll - sPrevMotion.roll) * t;
  m.pitch = sPrevMotion.pitch + (sMotion.pitch - sPrevMotion.pitch) * t;
  m.sway = sPrevMotion.sway + (sMotion.sway - sPrevMotion.sway) * t;
  m.bob = sPrevMotion.bob + (sMotion.bob - sPrevMotion.bob) * t;
  return m;
}

void ResetMotion() {
  sPrevMotion = Motion();
  sMotion = Motion();
  sMotionValid = false;
  sLean = sTip = sStrideAmp = sPhase = sDip = sDipVel = sPrevFallSpeed = sBend = 0.f;
  sWasOnGround = true;
  sHavePrevForward = false;
}


// ---- Echoes' animations ------------------------------------------------------------------------

// Read from the user's Echoes image on a thread of its own, once per image.
struct EchoesLoad {
  std::mutex mutex;
  std::string path;
  bool loading = false;
  std::string error;
  std::shared_ptr< const PortEchoesLocomotion::Library > library;
  std::string status; // for the settings, written each tick
};

// Never destroyed: the loading thread may still hold it at exit.
EchoesLoad& Echoes() {
  static EchoesLoad* load = new EchoesLoad;
  return *load;
}

void RequestEchoes(const std::string& path) {
  EchoesLoad& load = Echoes();
  std::lock_guard lock(load.mutex);
  if (load.path == path) {
    return;
  }
  load.path = path;
  load.loading = true;
  load.error.clear();
  load.library.reset();
  PortLog::Write("third person: reading Echoes' animations from %s\n", path.c_str());
  std::thread([path] {
    PortEchoesProbe::Skeleton skeleton;
    std::vector< std::pair< std::string, std::vector< uint8_t > > > clips;
    std::string error;
    std::shared_ptr< PortEchoesLocomotion::Library > library;
    if (PortEchoesProbe::LoadEchoesSamus(path, PortEchoesLocomotion::ClipNames(), skeleton, clips, error)) {
      auto built = std::make_shared< PortEchoesLocomotion::Library >();
      if (PortEchoesLocomotion::BuildLibrary(skeleton, clips, *built, error)) {
        library = std::move(built);
      }
    }
    EchoesLoad& load = Echoes();
    std::lock_guard lock(load.mutex);
    if (load.path != path) {
      return;
    }
    load.loading = false;
    load.library = library;
    load.error = library != nullptr ? std::string() : error;
    if (library != nullptr) {
      std::string missing;
      for (const std::string& name : library->missing) {
        missing += " " + name;
      }
      PortLog::Write("third person: Echoes' animations ready (%zu clips; not found:%s)\n", library->clips.size(),
                     missing.empty() ? " none" : missing.c_str());
    } else {
      PortLog::Write("third person: Echoes' animations not loaded: %s\n", error.c_str());
    }
  }).detach();
}

PortEchoesLocomotion::Player sEchoesPlayer;
std::shared_ptr< const PortEchoesLocomotion::Library > sEchoesLibrary;
const CAnimData* sEchoesFor = nullptr;
std::vector< CSegId > sEchoesSegs; // by Prime bone, as handed to the player
std::vector< CQuaternion > sEchoesBaseRotation;
std::vector< CVector3f > sEchoesBaseOffset;
bool sEchoesDriving = false;
int sPendingShot = -1; // the strongest shot since the last tick, for the recoil

void SetEchoesStatus(const std::string& status) {
  EchoesLoad& load = Echoes();
  std::lock_guard lock(load.mutex);
  load.status = status;
}

// Prime's skeleton as the body's layout has it, by name.
PortEchoesProbe::Skeleton PrimeSkeleton(const CCharLayoutInfo& layout, std::vector< CSegId >& segs) {
  PortEchoesProbe::Skeleton skeleton;
  segs.clear();
  for (auto it = layout.PortNameMap().begin(); it != layout.PortNameMap().end(); ++it) {
    const CSegId seg = it->second;
    if (seg == CSegId::Invalid() || seg == CSegId::Null()) {
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
    segs.push_back(seg);
  }
  return skeleton;
}

// Puts the body's own pose back.
void StopEchoes(CAnimData* anim) {
  if (sEchoesDriving && anim != nullptr && anim == sEchoesFor) {
    CHierarchyPoseBuilder& pose = anim->PoseBuilder();
    for (size_t i = 0; i < sEchoesSegs.size() && i < sEchoesBaseRotation.size(); ++i) {
      pose.Insert(sEchoesSegs[i], sEchoesBaseRotation[i]);
      pose.Insert(sEchoesSegs[i], sEchoesBaseOffset[i]);
    }
    anim->SetPoseBuilderValid(false);
  }
  if (sEchoesDriving) {
    sEchoesPlayer.Reset();
  }
  sEchoesDriving = false;
}

// Poses the body from Echoes' animations; false while they are not loaded.
// What the player did this tick, beyond moving, for Echoes' animations.
struct EchoesEvents {
  float turnRate = 0.f; // radians a second, left positive
  bool jumped = false;
  bool doubleJumped = false;
  bool dashing = false;
  int morph = 0; // 1 curling into the ball, 2 standing up out of it
};
bool sEchoesWasOnGround = true;
float sEchoesPrevUpSpeed = 0.f;

bool DriveEchoes(CAnimData& anim, const CPlayer& player, float dt, float forwardSpeed, float sideSpeed,
                 bool onGround, float aimPitch, float aimYaw, const EchoesEvents& events,
                 const PortDebug::ThirdPersonSettings& s) {
  const std::string path = PortDebug::EchoesImagePath();
  if (path.empty()) {
    SetEchoesStatus("Pick your Echoes image with the button above first.");
    return false;
  }
  RequestEchoes(path);
  std::shared_ptr< const PortEchoesLocomotion::Library > library;
  {
    EchoesLoad& load = Echoes();
    std::lock_guard lock(load.mutex);
    if (load.library == nullptr) {
      load.status = load.loading ? "Reading Echoes' animations..." : "Echoes' animations: " + load.error;
      return false;
    }
    library = load.library;
  }
  if (library != sEchoesLibrary) {
    StopEchoes(&anim);
    sEchoesLibrary = library;
    sEchoesPlayer.SetLibrary(library);
  }
  if (sEchoesFor != &anim) {
    if (sEchoesFor != nullptr) {
      sEchoesDriving = false; // the old body is gone; nothing to put back
    }
    const CCharLayoutInfo* layout = anim.GetCharLayoutInfo();
    if (layout == nullptr) {
      return false;
    }
    sEchoesFor = &anim;
    const PortEchoesProbe::Skeleton prime = PrimeSkeleton(*layout, sEchoesSegs);
    sEchoesPlayer.SetPrime(prime);
    int mapped = 0;
    for (const PortEchoesProbe::Bone& b : prime.bones) {
      mapped += library->echoes.Find(b.name) != nullptr;
    }
    PortLog::Write("third person: Echoes' animations drive %d of the body's %zu bones; aim turns the cannon "
                   "up %.0f, down %.0f, left %.0f, right %.0f degrees at full weight\n",
                   mapped, prime.bones.size(), sEchoesPlayer.AimRange(0), sEchoesPlayer.AimRange(1),
                   sEchoesPlayer.AimRange(2), sEchoesPlayer.AimRange(3));
  }

  // The pose tree as the body's own animation has it, kept to put back.
  anim.PreRender();
  CHierarchyPoseBuilder& pose = anim.PoseBuilder();
  if (!sEchoesDriving) {
    sEchoesBaseRotation.clear();
    sEchoesBaseOffset.clear();
    for (const CSegId& seg : sEchoesSegs) {
      sEchoesBaseRotation.push_back(pose.GetSegRotation(seg));
      sEchoesBaseOffset.push_back(pose.PortGetSegOffset(seg));
    }
  }

  PortEchoesLocomotion::Input in;
  in.dt = dt;
  in.forwardSpeed = forwardSpeed;
  in.sideSpeed = sideSpeed;
  in.onGround = onGround;
  in.scale = std::max(player.GetModelScale().GetZ(), 0.01f);
  in.aim = s.torsoAim;
  in.aimPitch = aimPitch;
  in.aimYaw = aimYaw;
  in.turnRate = events.turnRate;
  in.jumped = events.jumped;
  in.doubleJumped = events.doubleJumped;
  in.dashing = events.dashing;
  in.morph = events.morph;
  if (sPendingShot >= 0) {
    sEchoesPlayer.Shoot(sPendingShot);
    sPendingShot = -1;
  }
  const PortEchoesAnim::PrimePose& p = sEchoesPlayer.Update(in);
  for (size_t i = 0; i < sEchoesSegs.size() && i < p.locals.size(); ++i) {
    const PortEchoesAnim::Quat& q = p.locals[i];
    pose.Insert(sEchoesSegs[i], CQuaternion(q.w, q.x, q.y, q.z));
    if (p.hasOffset[i]) {
      const PortEchoesAnim::Vec3& o = p.offsets[i];
      pose.Insert(sEchoesSegs[i], CVector3f(o.x, o.y, o.z));
    }
  }
  anim.SetPoseBuilderValid(false);
  sEchoesDriving = true;

  std::string status = "Playing Echoes' animations.";
  if (!library->missing.empty()) {
    status += " Not found:";
    for (const std::string& name : library->missing) {
      status += " " + name;
    }
  }
  SetEchoesStatus(status);
  return true;
}

} // namespace

void UpdateFrame(const CStateManager& mgr) {
  const float dt = FrameDt();
  const PortDebug::ThirdPersonSettings s = PortDebug::ThirdPerson();
  const CPlayer* player = mgr.GetPlayer();
  const CCameraManager* cameras = mgr.GetCameraManager();
  // Only the game's own first-person camera is moved; every other camera keeps its view, and a
  // switch to one cuts straight to it (blending its view towards an offset made for another would
  // be wrong).
  const bool fpCamera = player != nullptr && cameras != nullptr && !PortFreeCam::Active() &&
                        cameras->IsInFPCamera() && !cameras->IsInCinematicCamera() &&
                        player->GetCameraState() == CPlayer::kCS_FirstPerson &&
                        player->GetMorphballTransitionState() != CPlayer::kMS_Morphed;
  if (!fpCamera) {
    sWeight = 0.f;
    sActive = false;
    sShowBody = false;
    sFresh = true;
    return;
  }
  // Switched on: eases back over about a third of a second. Switched off: eases in to the eyes.
  sWeight = Clamp(sWeight + (s.enabled ? dt : -dt) / 0.3f, 0.f, 1.f);
  if (sWeight <= 0.f) {
    sActive = false;
    sShowBody = false;
    sFresh = true;
    return;
  }

  const CTransform4f game = cameras->GetCurrentCameraTransform(mgr);
  const CVector3f eye = game.GetTranslation();
  CVector3f forward = game.GetForward();
  CVector3f right = game.GetRight();
  if (!forward.CanBeNormalized() || !right.CanBeNormalized()) {
    sActive = false;
    sShowBody = false;
    return;
  }
  forward = forward.AsNormalized();
  right = right.AsNormalized();

  // Where the crosshair ray meets the world, or a locked-on target. The camera looks at that point
  // so the centre of the screen is where a shot from the eyes goes, whatever the shoulder offset.
  float converge = RayDistance(mgr, eye, forward, 80.f);
  if (const CActor* target =
          TCastToConstPtr< CActor >(mgr.GetObjectById(player->GetOrbitTargetId()))) {
    const float d = CVector3f::Dot(target->GetAimPosition(mgr, 0.f) - eye, forward);
    if (d > 0.f) {
      converge = std::min(converge, d);
    }
  }
  converge = std::max(converge, 3.f);

  // The camera's arm: back along the view, out to the shoulder and up, kept short of walls.
  const CVector3f arm = forward * -s.distance + right * s.shoulder + CVector3f(0.f, 0.f, s.height);
  const float armLength = arm.Magnitude();
  float allowed = 0.f;
  CVector3f armDir = CVector3f(0.f, 0.f, 1.f);
  if (armLength > 0.01f) {
    armDir = arm * (1.f / armLength);
    // Stops a little short of the wall, so the near plane does not cut into it.
    allowed = std::max(RayDistance(mgr, eye, armDir, armLength + 0.35f) - 0.35f, 0.f);
  }
  if (sFresh) {
    sBoom = allowed;
    sConverge = converge;
    sFresh = false;
  }
  // Into a wall at once (never through it); back out gently.
  sBoom = allowed < sBoom ? allowed : sBoom + (allowed - sBoom) * Approach(dt, 4.f);
  sConverge += (converge - sConverge) * Approach(dt, 10.f);

  // Smoothstep, so the ease in has no corner at either end.
  const float w = sWeight * sWeight * (3.f - 2.f * sWeight);
  const CVector3f camPos = eye + armDir * (sBoom * w);
  const CVector3f look = eye + forward * sConverge;
  sView = CTransform4f::LookAt(camPos, look, CVector3f(0.f, 0.f, 1.f));
  sActive = true;
  // Pulled into her by a wall, the camera would be inside the helmet: leave the body out then.
  sShowBody = sBoom * w > 0.9f;
}

void Tick(float dt, CPlayer& player, CStateManager& mgr) {
  const PortDebug::ThirdPersonSettings s = PortDebug::ThirdPerson();
  CAnimData* anim = player.HasAnimation() ? player.AnimationData() : nullptr;
  const CPlayer::EPlayerMorphBallState morphState = player.GetMorphballTransitionState();
  const bool upright = morphState == CPlayer::kMS_Unmorphed;
  // Echoes' animations, already playing, see the morph ball's way in and out through.
  const bool morphing = (morphState == CPlayer::kMS_Morphing || morphState == CPlayer::kMS_Unmorphing) &&
                        s.echoesAnims && sEchoesDriving;
  if (!s.enabled || !(upright || morphing) || dt <= 0.f) {
    if (anim != nullptr) {
      RestoreSpine(*anim);
    }
    StopEchoes(anim);
    sPendingShot = -1;
    if (!s.echoesAnims || !s.enabled) {
      SetEchoesStatus("");
    }
    ResetMotion();
    return;
  }

  const CTransform4f& xf = player.GetTransform();
  CVector3f forward = xf.GetForward();
  forward = CVector3f(forward.GetX(), forward.GetY(), 0.f);
  if (!forward.CanBeNormalized()) {
    return;
  }
  forward = forward.AsNormalized();
  const CVector3f right(forward.GetY(), -forward.GetX(), 0.f);
  const CVector3f v = player.GetVelocityWR();
  const float forwardSpeed = CVector3f::Dot(v, forward);
  const float sideSpeed = CVector3f::Dot(v, right);
  const float groundSpeed = std::sqrt(forwardSpeed * forwardSpeed + sideSpeed * sideSpeed);
  const bool onGround = player.GetPlayerMovementState() == NPlayer::kMS_OnGround;

  // Turn rate, left positive, from the heading's change this tick.
  float turnRate = 0.f;
  if (sHavePrevForward) {
    const float cross = sPrevForward.GetX() * forward.GetY() - sPrevForward.GetY() * forward.GetX();
    turnRate = std::asin(Clamp(cross, -1.f, 1.f)) / dt;
  }
  sPrevForward = forward;
  sHavePrevForward = true;

  // Echoes' animations, when on and loaded, pose the whole body; the hand-made motion stands down.
  bool echoes = false;
  if (anim != nullptr && s.echoesAnims) {
    if (!sEchoesDriving) {
      RestoreSpine(*anim);
    }
    // Where the cannon points: the first-person camera's pitch, and its heading against the body's
    // (they part with free aim and lock-on).
    float aimPitch = 0.f;
    float aimYaw = 0.f;
    CVector3f view = mgr.GetCameraManager()->GetCurrentCameraTransform(mgr).GetForward();
    if (view.CanBeNormalized()) {
      view = view.AsNormalized();
      aimPitch = std::asin(Clamp(view.GetZ(), -1.f, 1.f));
      aimYaw = std::atan2(CVector3f::Dot(view, right), CVector3f::Dot(view, forward));
    }
    EchoesEvents events;
    events.turnRate = turnRate;
    // A jump leaves the ground going up; a double jump kicks the climb back up in the air.
    const float upSpeed = v.GetZ();
    events.jumped = sEchoesWasOnGround && !onGround && upSpeed > 1.f;
    events.doubleJumped = !sEchoesWasOnGround && !onGround && upSpeed - sEchoesPrevUpSpeed > 3.f;
    sEchoesWasOnGround = onGround;
    sEchoesPrevUpSpeed = upSpeed;
    events.dashing = player.IsSidewaysDashing();
    events.morph = morphState == CPlayer::kMS_Morphing ? 1 : morphState == CPlayer::kMS_Unmorphing ? 2 : 0;
    echoes = DriveEchoes(*anim, player, dt, forwardSpeed, sideSpeed, onGround, aimPitch, aimYaw, events, s);
  } else {
    StopEchoes(anim);
    SetEchoesStatus("");
  }
  sPendingShot = -1;

  // Samus runs at about 16 units a second; the motion is scaled to that.
  const float run = 16.f;
  sPrevMotion = sMotion;
  Motion m;
  if (s.bodyMotion && !echoes) {
    // Into turns (left turn: tips left) and strafes (moving right: tips right), and forward with
    // speed; in the air back on the way up and forward on the way down.
    const float leanTarget =
        Clamp(-turnRate * 0.09f + sideSpeed / run * 6.f * kDeg, -10.f * kDeg, 10.f * kDeg);
    float tipTarget = -Clamp(forwardSpeed / run, -1.f, 1.f) * 5.f * kDeg;
    if (!onGround) {
      tipTarget = Clamp(v.GetZ() / 20.f, -1.f, 1.f) * 6.f * kDeg;
    }
    sLean += (leanTarget - sLean) * Approach(dt, 8.f);
    sTip += (tipTarget - sTip) * Approach(dt, 6.f);

    // A stride every 1.8 units travelled: two bobs and one sway each, and a small roll with the
    // sway. Fades in and out with speed, and stops in the air.
    const float ampTarget = onGround ? Clamp(groundSpeed / run, 0.f, 1.f) : 0.f;
    sStrideAmp += (ampTarget - sStrideAmp) * Approach(dt, 6.f);
    sPhase = std::fmod(sPhase + dt * groundSpeed / 1.8f * 2.f * kPi, 2.f * kPi);

    // Landing: a dip that grows with the fall, sprung back up.
    if (onGround && !sWasOnGround) {
      sDipVel -= Clamp(sPrevFallSpeed / 25.f, 0.f, 1.f) * 1.6f;
    }
    sPrevFallSpeed = std::max(-v.GetZ(), 0.f);
    sWasOnGround = onGround;
    const float k = 220.f;
    const float c = 2.f * std::sqrt(k) * 0.8f;
    sDipVel += (-k * sDip - c * sDipVel) * dt;
    sDip += sDipVel * dt;
    sDip = Clamp(sDip, -0.25f, 0.1f);

    m.roll = sLean + std::sin(sPhase) * 1.5f * kDeg * sStrideAmp;
    m.pitch = sTip;
    m.sway = std::sin(sPhase) * 0.035f * sStrideAmp;
    m.bob = (std::fabs(std::sin(sPhase)) - 0.64f) * 0.07f * sStrideAmp + sDip;
  }
  sMotion = m;
  sMotionGeneration = CActor::PortTickGeneration();
  if (!sMotionValid) {
    sPrevMotion = sMotion;
    sMotionValid = true;
  }

  // The torso bends towards the aim: up to half the view's pitch, eased.
  if (anim != nullptr) {
    if (echoes) {
      sBend = 0.f; // Echoes' aim animations do it
    } else if (s.torsoAim && sActive) {
      const CVector3f view = mgr.GetCameraManager()->GetCurrentCameraTransform(mgr).GetForward();
      const float aimPitch =
          std::asin(Clamp(view.CanBeNormalized() ? view.AsNormalized().GetZ() : 0.f, -1.f, 1.f));
      const float target = Clamp(aimPitch * 0.5f, -35.f * kDeg, 35.f * kDeg);
      sBend += (target - sBend) * Approach(dt, 10.f);
      BendSpine(*anim, sBend);
    } else {
      sBend = 0.f;
      RestoreSpine(*anim);
    }
  }
}

bool Active() { return sActive; }

void NoteShot(int strength) { sPendingShot = std::max(sPendingShot, strength); }

std::string EchoesAnimationStatus() {
  EchoesLoad& load = Echoes();
  std::lock_guard lock(load.mutex);
  return load.status;
}

CTransform4f View(const CTransform4f& game) { return sActive ? sView : game; }

bool ShowBody() { return sActive && sShowBody; }

bool BodyView(const CPlayer& player, const CTransform4f& view, CTransform4f& out) {
  if (!ShowBody() || !sMotionValid || !PortDebug::ThirdPerson().bodyMotion) {
    return false;
  }
  const Motion m = PresentedMotion();
  // The motion in the body's own frame, about its feet...
  const CTransform4f local = CTransform4f::Translate(m.sway, 0.f, m.bob) *
                             CTransform4f::RotateY(CRelAngle::FromRadians(m.roll)) *
                             CTransform4f::RotateX(CRelAngle::FromRadians(m.pitch));
  // ...carried into the world about the simulation's transform. The actor's own render scope then
  // moves the result onto its presented pose, as it does the body without this.
  const CTransform4f& cur = player.GetTransform();
  const CTransform4f motion = cur * local * cur.GetQuickInverse();
  // Drawn = view^-1 * motion * world, as CActor::PortPresentedView does it.
  out = motion.GetQuickInverse() * view;
  return true;
}

} // namespace PortThirdPerson
