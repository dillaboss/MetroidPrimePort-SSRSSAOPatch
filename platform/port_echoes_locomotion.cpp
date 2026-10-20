// Echoes' Samus locomotion on Prime's Samus (see port_echoes_locomotion.h).
#include "port_echoes_locomotion.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace PortEchoesLocomotion {
namespace {

using PortEchoesAnim::Clip;
using PortEchoesAnim::PrimePose;
using PortEchoesAnim::Quat;
using PortEchoesAnim::Vec3;

const char* const kIdle = "B_ready_samus";
const char* const kAir = "B_jumploop_samus";
const char* const kLand = "B_jumpland_toready_samus";
const char* const kAims[4] = {"Aim_U_additive_samus", "Aim_D_additive_samus",
                              "Aim_L_additive_samus", "Aim_R_additive_samus"};
const char* const kShots[3] = {"A_shoot_additive_samus", "A_mediumshoot_additive_samus",
                               "A_heavyshoot_additive_samus"};
constexpr float kPi = 3.14159265f;
// Forward, backward, left, right; each slowest first.
const char* const kSets[4][3] = {
    {"B_walk_samus", "B_jog_samus", "B_run_samus"},
    {"B_walk_backwards_samus", "B_jog_backward_samus", "B_run_backwards_samus"},
    {"B_strafe_Left_walk_samus", "B_strafe_Left_slow_samus", "B_strafe_Left_samus"},
    {"B_strafe_Right_walk_samus", "B_strafe_Right_slow_samus", "B_strafe_Right_samus"},
};

float Approach(float dt, float rate) { return 1.f - std::exp(-rate * dt); }

// How far a clip's root travels a second, in skeleton units (the clips carry their own travel on
// the root bone).
float ClipSpeed(const PortEchoesProbe::Skeleton& echoes, const Clip& clip) {
  const PortEchoesProbe::Bone* root = echoes.Find("root");
  if (root == nullptr || !(clip.duration > 0.f)) {
    return 0.f;
  }
  for (const PortEchoesAnim::Channel& ch : clip.channels) {
    if (ch.bone == root->id && ch.translations.size() >= 2) {
      const Vec3& a = ch.translations.front();
      const Vec3& b = ch.translations.back();
      return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)) / clip.duration;
    }
  }
  return 0.f;
}

float Wrap(float time, float duration) {
  if (!(duration > 0.f)) {
    return 0.f;
  }
  const float t = std::fmod(time, duration);
  return t < 0.f ? t + duration : t;
}

// A weighted average of poses: rotations summed on one side of the sphere and normalised, offsets
// averaged.
void Blend(const std::vector< std::pair< PrimePose, float > >& poses, PrimePose& out) {
  float total = 0.f;
  for (const auto& p : poses) {
    total += p.second;
  }
  if (poses.empty() || !(total > 0.f)) {
    return;
  }
  const size_t n = poses.front().first.locals.size();
  out = poses.front().first;
  for (size_t i = 0; i < n; ++i) {
    Quat acc{0.f, 0.f, 0.f, 0.f};
    Vec3 off;
    bool hasOffset = false;
    const Quat& ref = poses.front().first.locals[i];
    for (const auto& [pose, weight] : poses) {
      const float w = weight / total;
      Quat q = pose.locals[i];
      if (q.w * ref.w + q.x * ref.x + q.y * ref.y + q.z * ref.z < 0.f) {
        q = Quat{-q.w, -q.x, -q.y, -q.z};
      }
      acc.w += q.w * w;
      acc.x += q.x * w;
      acc.y += q.y * w;
      acc.z += q.z * w;
      off.x += pose.offsets[i].x * w;
      off.y += pose.offsets[i].y * w;
      off.z += pose.offsets[i].z * w;
      hasOffset = hasOffset || pose.hasOffset[i];
    }
    out.locals[i] = PortEchoesAnim::Normalized(acc);
    out.offsets[i] = off;
    out.hasOffset[i] = hasOffset;
  }
}

// Each bone's place in skeleton space for a pose on Prime's skeleton.
std::vector< Vec3 > PrimePositions(const PortEchoesProbe::Skeleton& prime, const PrimePose& pose) {
  const size_t n = prime.bones.size();
  std::map< uint32_t, size_t > index;
  for (size_t i = 0; i < n; ++i) {
    index[prime.bones[i].id] = i;
  }
  std::vector< Vec3 > positions(n);
  std::vector< Quat > orientations(n);
  std::vector< int > done(n, 0);
  const std::function< void(size_t, int) > place = [&](size_t i, int depth) {
    if (done[i] != 0 || depth > 200) {
      return;
    }
    const PortEchoesProbe::Bone& b = prime.bones[i];
    const auto p = index.find(b.parent);
    const bool hasParent = p != index.end() && p->second != i;
    Quat qp;
    Vec3 pp;
    Vec3 d{b.position[0], b.position[1], b.position[2]};
    if (hasParent) {
      place(p->second, depth + 1);
      qp = orientations[p->second];
      pp = positions[p->second];
      const float* q = prime.bones[p->second].position;
      d = Vec3{d.x - q[0], d.y - q[1], d.z - q[2]};
    }
    if (pose.hasOffset[i]) {
      d = pose.offsets[i];
    }
    const Vec3 r = PortEchoesAnim::Rotate(qp, d);
    positions[i] = Vec3{pp.x + r.x, pp.y + r.y, pp.z + r.z};
    orientations[i] = PortEchoesAnim::Normalized(PortEchoesAnim::Mul(qp, pose.locals[i]));
    done[i] = 1;
  };
  for (size_t i = 0; i < n; ++i) {
    place(i, 0);
  }
  return positions;
}

float SmoothStep(float x) {
  x = std::clamp(x, 0.f, 1.f);
  return x * x * (3.f - 2.f * x);
}

} // namespace

const std::vector< std::string >& ClipNames() {
  static const std::vector< std::string > names = [] {
    std::vector< std::string > v = {kIdle, kAir, kLand};
    for (const char* name : kAims) {
      v.push_back(name);
    }
    for (const char* name : kShots) {
      v.push_back(name);
    }
    for (const auto& set : kSets) {
      for (const char* name : set) {
        v.push_back(name);
      }
    }
    return v;
  }();
  return names;
}

bool BuildLibrary(const PortEchoesProbe::Skeleton& echoes,
                  const std::vector< std::pair< std::string, std::vector< uint8_t > > >& data,
                  Library& out, std::string& error) {
  out = Library{};
  out.echoes = echoes;
  for (const auto& [name, bytes] : data) {
    Clip clip;
    std::string why;
    if (bytes.empty() || !PortEchoesAnim::ParseClip(bytes.data(), bytes.size(), clip, why)) {
      out.missing.push_back(name);
      continue;
    }
    out.clips[name] = std::move(clip);
  }
  if (out.clips.count(kIdle) == 0 || out.clips.count(kSets[0][0]) == 0) {
    error = "Echoes' standing and walking animations were not found";
    return false;
  }
  return true;
}

void Player::SetLibrary(std::shared_ptr< const Library > library) {
  mLibrary = std::move(library);
  Classify();
  Calibrate();
  Reset();
}

void Player::SetPrime(const PortEchoesProbe::Skeleton& prime) {
  mPrime = prime;
  mRootBone = -1;
  for (size_t i = 0; i < mPrime.bones.size(); ++i) {
    if (mPrime.bones[i].name == "root") {
      mRootBone = int(i);
    }
  }
  Calibrate();
  Reset();
}

void Player::Reset() {
  mForward = mSide = mPhase = mIdleTime = mAirTime = mAirClipTime = mAirWeight = 0.f;
  mLandTime = -1.f;
  mShotTime = -1.f;
  mAimPitch = mAimYaw = 0.f;
  mFresh = true;
  mLast.clear();
}

void Player::Classify() {
  for (auto& set : mSets) {
    set.clear();
  }
  mIdle = mAir = mLand = nullptr;
  if (mLibrary == nullptr) {
    return;
  }
  const auto find = [this](const char* name) -> const Clip* {
    const auto it = mLibrary->clips.find(name);
    return it == mLibrary->clips.end() ? nullptr : &it->second;
  };
  mIdle = find(kIdle);
  mAir = find(kAir);
  mLand = find(kLand);
  for (int k = 0; k < 4; ++k) {
    mAim[k] = find(kAims[k]);
  }
  for (int k = 0; k < 3; ++k) {
    mShots[k] = find(kShots[k]);
  }
  for (int k = 0; k < 4; ++k) {
    for (const char* name : kSets[k]) {
      const Clip* clip = find(name);
      const float speed = clip != nullptr ? ClipSpeed(mLibrary->echoes, *clip) : 0.f;
      if (clip != nullptr && speed > 0.05f) {
        mSets[k].push_back({clip, name, speed});
      }
    }
    std::sort(mSets[k].begin(), mSets[k].end(),
              [](const Moving& a, const Moving& b) { return a.speed < b.speed; });
  }
}

// Which key of each aim animation holds the full aim, and how far it turns the arm cannon: the
// cannon's direction (elbow to charge locator) on the standing pose with and without it.
void Player::Calibrate() {
  mPrimeOfEchoes.clear();
  for (int k = 0; k < 4; ++k) {
    mAimTime[k] = 0.f;
    mAimRange[k] = 0.f;
  }
  if (!Ready()) {
    return;
  }
  for (const PortEchoesProbe::Bone& e : mLibrary->echoes.bones) {
    for (size_t i = 0; i < mPrime.bones.size(); ++i) {
      if (mPrime.bones[i].name == e.name) {
        mPrimeOfEchoes[e.id] = int(i);
      }
    }
  }
  int from = -1;
  int to = -1;
  for (size_t i = 0; i < mPrime.bones.size(); ++i) {
    const std::string& name = mPrime.bones[i].name;
    if (name == "R_elbow") {
      from = int(i);
    } else if (name == "Charge_LCTR" || (name == "GUN_LCTR" && to < 0)) {
      to = int(i);
    }
  }
  const Clip* standing =
      mIdle != nullptr ? mIdle : (mSets[0].empty() ? nullptr : mSets[0].front().clip);
  if (standing == nullptr || from < 0 || to < 0) {
    for (int k = 0; k < 4; ++k) {
      mAimRange[k] = mAim[k] != nullptr ? 45.f * kPi / 180.f : 0.f;
    }
    return;
  }
  const PrimePose base = PoseOf(*standing, 0.f);
  const auto angles = [&](const PrimePose& pose, float& pitch, float& yaw) {
    const std::vector< Vec3 > p = PrimePositions(mPrime, pose);
    const Vec3 d{p[size_t(to)].x - p[size_t(from)].x, p[size_t(to)].y - p[size_t(from)].y,
                 p[size_t(to)].z - p[size_t(from)].z};
    pitch = std::atan2(d.z, std::sqrt(d.x * d.x + d.y * d.y));
    yaw = std::atan2(d.x, d.y);
  };
  float basePitch = 0.f;
  float baseYaw = 0.f;
  angles(base, basePitch, baseYaw);
  for (int k = 0; k < 4; ++k) {
    if (mAim[k] == nullptr) {
      continue;
    }
    for (float t : {0.f, mAim[k]->duration}) {
      PrimePose pose = base;
      ApplyAdditive(*mAim[k], t, 1.f, pose);
      float pitch = 0.f;
      float yaw = 0.f;
      angles(pose, pitch, yaw);
      float turn = k < 2 ? pitch - basePitch : yaw - baseYaw;
      turn = std::fabs(std::remainder(turn, 2.f * kPi));
      if (turn > mAimRange[k]) {
        mAimRange[k] = turn;
        mAimTime[k] = t;
      }
    }
    // An animation that hardly turns the cannon is still played across a usual range.
    if (mAimRange[k] < 3.f * kPi / 180.f) {
      mAimRange[k] = 45.f * kPi / 180.f;
    }
  }
}

float Player::AimRange(int direction) const {
  return direction >= 0 && direction < 4 ? mAimRange[direction] * 180.f / kPi : 0.f;
}

void Player::Shoot(int strength) {
  strength = std::clamp(strength, 0, 2);
  // The nearest recoil there is.
  for (int k : {strength, strength == 0 ? 1 : strength - 1, 0, 1, 2}) {
    if (mShots[k] != nullptr) {
      mShotStrength = k;
      mShotTime = 0.f;
      return;
    }
  }
}

void Player::ApplyAdditive(const Clip& clip, float time, float weight, PrimePose& pose) const {
  if (weight <= 0.f) {
    return;
  }
  for (const PortEchoesAnim::Channel& ch : clip.channels) {
    if (ch.rotations.empty()) {
      continue;
    }
    const auto it = mPrimeOfEchoes.find(ch.bone);
    // The root's turn is the clip's travel, which the player owns.
    if (it == mPrimeOfEchoes.end() || it->second == mRootBone ||
        size_t(it->second) >= pose.locals.size()) {
      continue;
    }
    const PortEchoesAnim::Sample s = PortEchoesAnim::SampleChannel(clip, ch, time);
    const Quat delta =
        weight >= 1.f ? s.rotation : PortEchoesAnim::Slerp(Quat{}, s.rotation, weight);
    Quat& local = pose.locals[size_t(it->second)];
    local = PortEchoesAnim::Normalized(PortEchoesAnim::Mul(local, delta));
  }
}

PrimePose Player::PoseOf(const Clip& clip, float time) const {
  const PortEchoesAnim::Convention& c = PortEchoesAnim::Conventions().front();
  return PortEchoesAnim::RetargetToPrime(
      mLibrary->echoes, PortEchoesAnim::EchoesPose(mLibrary->echoes, clip, time, c), mPrime);
}

const PrimePose& Player::Update(const Input& in) {
  if (!Ready()) {
    return mPose;
  }
  const float dt = std::clamp(in.dt, 0.f, 0.1f);
  const bool fresh = mFresh;
  const float scale = std::max(in.scale, 0.01f);
  const float targetForward = in.forwardSpeed / scale;
  const float targetSide = in.sideSpeed / scale;
  if (mFresh) {
    mForward = targetForward;
    mSide = targetSide;
  } else {
    mForward += (targetForward - mForward) * Approach(dt, 10.f);
    mSide += (targetSide - mSide) * Approach(dt, 10.f);
  }
  const float speed = std::sqrt(mForward * mForward + mSide * mSide);

  // Direction: forward and backward against the sides, squared cosines so they sum to one.
  float dir[4] = {1.f, 0.f, 0.f, 0.f};
  if (speed > 1e-3f) {
    const float c = mForward / speed;
    const float s = mSide / speed;
    dir[0] = std::max(c, 0.f) * std::max(c, 0.f);
    dir[1] = std::max(-c, 0.f) * std::max(-c, 0.f);
    dir[2] = std::max(-s, 0.f) * std::max(-s, 0.f);
    dir[3] = std::max(s, 0.f) * std::max(s, 0.f);
  }
  // A direction without animations falls back to walking forward.
  for (int k = 1; k < 4; ++k) {
    if (mSets[k].empty()) {
      dir[0] += dir[k];
      dir[k] = 0.f;
    }
  }
  const float walkSpeed = mSets[0].empty() ? 1.f : mSets[0].front().speed;
  const float idleWeight =
      mIdle != nullptr ? std::clamp(1.f - speed / (0.6f * walkSpeed), 0.f, 1.f) : 0.f;

  std::vector< Contribution > base;
  float freq = 0.f;
  float freqWeight = 0.f;
  for (int k = 0; k < 4; ++k) {
    const float w = dir[k] * (1.f - idleWeight);
    if (w <= 1e-4f || mSets[k].empty()) {
      continue;
    }
    const std::vector< Moving >& set = mSets[k];
    // The two clips around this speed, or the nearest one played faster or slower.
    size_t a = 0;
    while (a + 1 < set.size() && set[a + 1].speed < speed) {
      ++a;
    }
    float t = 0.f;
    float rate = 1.f;
    if (speed <= set.front().speed) {
      a = 0;
      rate = std::max(speed / set.front().speed, 0.5f);
    } else if (a + 1 < set.size()) {
      t = (speed - set[a].speed) / (set[a + 1].speed - set[a].speed);
    } else {
      rate = std::min(speed / set.back().speed, 2.5f);
    }
    const auto add = [&](const Moving& m, float weight) {
      if (weight <= 1e-4f) {
        return;
      }
      base.push_back({m.clip, m.name, 0.f, weight});
      freq += weight * rate / std::max(m.clip->duration, 0.05f);
      freqWeight += weight;
    };
    add(set[a], w * (1.f - t));
    if (t > 0.f) {
      add(set[a + 1], w * t);
    }
  }
  if (freqWeight > 0.f) {
    mPhase = Wrap(mPhase + dt * freq / freqWeight, 1.f);
  }
  for (Contribution& c : base) {
    c.time = mPhase * c.clip->duration;
  }
  mIdleTime += dt;
  if (idleWeight > 0.f) {
    base.push_back({mIdle, kIdle, Wrap(mIdleTime, mIdle->duration), idleWeight});
  }

  // In the air after a moment off the ground; a landing after a real fall.
  if (!in.onGround) {
    mAirTime += dt;
  } else {
    if (mAirTime > 0.3f && mLand != nullptr) {
      mLandTime = 0.f;
    }
    mAirTime = 0.f;
  }
  const float airTarget = (!in.onGround && mAirTime > 0.08f && mAir != nullptr) ? 1.f : 0.f;
  mAirWeight = mFresh ? airTarget : mAirWeight + (airTarget - mAirWeight) * Approach(dt, 12.f);
  mAirClipTime = mAirWeight > 0.f ? mAirClipTime + dt : 0.f;
  float landWeight = 0.f;
  if (mLandTime >= 0.f && mLand != nullptr) {
    mLandTime += dt;
    const float progress = mLandTime / std::max(mLand->duration, 0.05f);
    if (progress >= 1.f) {
      mLandTime = -1.f;
    } else {
      const float moving = std::clamp(speed / walkSpeed, 0.f, 1.f);
      landWeight = (1.f - progress) * (1.f - progress) * (1.f - moving);
    }
  }
  mFresh = false;

  mLast.clear();
  const float baseShare = (1.f - mAirWeight) * (1.f - landWeight);
  for (Contribution& c : base) {
    c.weight *= baseShare;
    if (c.weight > 1e-4f) {
      mLast.push_back(c);
    }
  }
  if (landWeight > 0.f) {
    mLast.push_back(
        {mLand, kLand, std::min(mLandTime, mLand->duration), (1.f - mAirWeight) * landWeight});
  }
  if (mAirWeight > 1e-4f) {
    mLast.push_back({mAir, kAir, Wrap(mAirClipTime, mAir->duration), mAirWeight});
  }
  if (mLast.empty() && mIdle != nullptr) {
    mLast.push_back({mIdle, kIdle, Wrap(mIdleTime, mIdle->duration), 1.f});
  }

  std::vector< std::pair< PrimePose, float > > poses;
  for (const Contribution& c : mLast) {
    poses.emplace_back(PoseOf(*c.clip, c.time), c.weight);
  }
  Blend(poses, mPose);
  // Aim, as Echoes layers it: up or down and left or right, each by how far the cannon points
  // that way against how far its animation turns it.
  if (in.aim) {
    const float rate = fresh ? 1.f : Approach(dt, 15.f);
    mAimPitch += (in.aimPitch - mAimPitch) * rate;
    mAimYaw += (in.aimYaw - mAimYaw) * rate;
  } else {
    mAimPitch = mAimYaw = 0.f;
  }
  const float aimAmount[4] = {std::max(mAimPitch, 0.f), std::max(-mAimPitch, 0.f),
                              std::max(-mAimYaw, 0.f), std::max(mAimYaw, 0.f)};
  for (int k = 0; k < 4; ++k) {
    if (mAim[k] != nullptr && mAimRange[k] > 0.f && aimAmount[k] > 0.f) {
      ApplyAdditive(*mAim[k], mAimTime[k], std::min(aimAmount[k] / mAimRange[k], 1.f), mPose);
    }
  }
  // A shot's recoil, fading out over its last part.
  if (mShotTime >= 0.f) {
    const Clip* shot = mShots[mShotStrength];
    const float duration = std::max(shot->duration, 0.05f);
    if (mShotTime >= duration) {
      mShotTime = -1.f;
    } else {
      const float fade = 1.f - SmoothStep((mShotTime / duration - 0.6f) / 0.4f);
      ApplyAdditive(*shot, mShotTime, fade, mPose);
      mShotTime += dt;
    }
  }

  // The player moves the body: the root keeps its height, not the clips' travel.
  if (mRootBone >= 0 && size_t(mRootBone) < mPose.offsets.size()) {
    mPose.offsets[size_t(mRootBone)].x = 0.f;
    mPose.offsets[size_t(mRootBone)].y = 0.f;
    mPose.hasOffset[size_t(mRootBone)] = true;
  }
  return mPose;
}

std::vector< std::pair< std::string, float > > Player::Weights() const {
  std::vector< std::pair< std::string, float > > out;
  for (const Contribution& c : mLast) {
    out.emplace_back(c.name, c.weight);
  }
  std::sort(out.begin(), out.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });
  return out;
}

} // namespace PortEchoesLocomotion
