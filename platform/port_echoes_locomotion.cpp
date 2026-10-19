// Echoes' Samus locomotion on Prime's Samus (see port_echoes_locomotion.h).
#include "port_echoes_locomotion.h"

#include <algorithm>
#include <cmath>

namespace PortEchoesLocomotion {
namespace {

using PortEchoesAnim::Clip;
using PortEchoesAnim::PrimePose;
using PortEchoesAnim::Quat;
using PortEchoesAnim::Vec3;

const char* const kIdle = "B_ready_samus";
const char* const kAir = "B_jumploop_samus";
const char* const kLand = "B_jumpland_toready_samus";
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

} // namespace

const std::vector< std::string >& ClipNames() {
  static const std::vector< std::string > names = [] {
    std::vector< std::string > v = {kIdle, kAir, kLand};
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
  std::vector< PortEchoesAnim::NamedClip > named;
  for (const auto& [name, clip] : out.clips) {
    named.push_back({name, &clip});
  }
  out.ranking = PortEchoesAnim::RankConventions(out.echoes, named);
  return true;
}

void Player::SetLibrary(std::shared_ptr< const Library > library) {
  mLibrary = std::move(library);
  Classify();
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
  Reset();
}

void Player::Reset() {
  mForward = mSide = mPhase = mIdleTime = mAirTime = mAirClipTime = mAirWeight = 0.f;
  mLandTime = -1.f;
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

PrimePose Player::PoseOf(const Clip& clip, float time) const {
  const auto& conventions = PortEchoesAnim::Conventions();
  const PortEchoesAnim::Convention& c = conventions[std::min(mConvention, conventions.size() - 1)];
  return PortEchoesAnim::RetargetToPrime(
      mLibrary->echoes, PortEchoesAnim::EchoesPose(mLibrary->echoes, clip, time, c), mPrime);
}

const PrimePose& Player::Update(const Input& in) {
  if (!Ready()) {
    return mPose;
  }
  const float dt = std::clamp(in.dt, 0.f, 0.1f);
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
