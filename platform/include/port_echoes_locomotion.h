#pragma once

#include "port_echoes_anim.h"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Echoes' Samus locomotion played on Prime's Samus in the third-person view: standing, walking,
// jogging and running forwards and backwards, strafing, turning on the spot, jumping (take-off,
// the air, double jumps, side dashes) and landing, and the morph ball's way in and out, blended by
// the player's motion and carried onto Prime's skeleton (port_echoes_anim.h). Faster than Echoes'
// clips move, the stride lengthens before the playback speeds up. On top, as
// Echoes layers them, additive animations: aiming up, down, left and right, and the recoil of a
// shot. Echoes' keys are read as Prime reads its own (the "prime" convention, which the report
// ranks first on real data): its bind rotations are for skinning only. Pure: no
// engine types, so it can be tested on its own; port_third_person.cpp feeds it the player's
// motion and puts its pose into the body's pose builder.
namespace PortEchoesLocomotion {

// The Echoes animations this plays, by name.
const std::vector< std::string >& ClipNames();

struct Library {
  PortEchoesProbe::Skeleton echoes;
  std::map< std::string, PortEchoesAnim::Clip > clips;
  std::vector< std::string > missing; // names not found or not decoded
};

// Decodes the clips. False, with `error`, when Samus cannot stand and walk with what was found.
bool BuildLibrary(const PortEchoesProbe::Skeleton& echoes,
                  const std::vector< std::pair< std::string, std::vector< uint8_t > > >& data,
                  Library& out, std::string& error);

struct Input {
  float dt = 0.f;
  float forwardSpeed = 0.f; // world units a second, in the body's frame
  float sideSpeed = 0.f;    // to the right
  bool onGround = true;
  float scale = 1.f;          // world units per skeleton unit (the body model's scale)
  bool aim = true;            // layer the aim
  float aimPitch = 0.f;       // radians, up positive
  float aimYaw = 0.f;         // radians from the body's forward, right positive
  float turnRate = 0.f;       // radians a second, left positive
  bool jumped = false;        // left the ground with a jump this tick
  bool doubleJumped = false;  // jumped again in the air this tick
  bool dashing = false;       // a side dash (left or right by sideSpeed)
  int morph = 0;              // 1 curling into the morph ball, 2 standing up out of it
  float morphProgress = -1.f; // the morph transition's progress, 0..1 (< 0: by time)
};

class Player {
public:
  void SetLibrary(std::shared_ptr< const Library > library);
  void SetPrime(const PortEchoesProbe::Skeleton& prime);
  bool Ready() const { return mLibrary != nullptr && !mPrime.bones.empty(); }
  void Reset();
  // Advances by `in.dt` and returns the pose on Prime's skeleton (indexed like the skeleton given
  // to SetPrime). The root keeps its height but not the clips' travel: the player moves the body.
  const PortEchoesAnim::PrimePose& Update(const Input& in);
  // A shot's recoil from the next Update on: 0 light, 1 medium, 2 heavy.
  void Shoot(int strength);
  // How far each aim animation turns the arm cannon at full weight, in degrees (up, down, left,
  // right; 0 without the animation), as measured on the standing pose.
  float AimRange(int direction) const;

  // What the blend is made of, for display: clip name and weight, heaviest first.
  std::vector< std::pair< std::string, float > > Weights() const;
  // How much longer than the clip's the last update's stride was (1: as animated).
  float LastStride() const { return mLastStride; }

private:
  struct Moving {
    const PortEchoesAnim::Clip* clip = nullptr;
    std::string name;
    float speed = 0.f; // skeleton units a second
  };
  struct Contribution {
    const PortEchoesAnim::Clip* clip;
    std::string name;
    float time;
    float weight;
    float stride = 1.f; // leg swing scaled about its mean
  };

  void Classify();
  void Calibrate();
  // Swings the legs further (stride > 1) about the clip's average leg pose.
  void Stretch(const PortEchoesAnim::Clip& clip, float stride,
               PortEchoesAnim::PrimePose& pose) const;
  PortEchoesAnim::PrimePose PoseOf(const PortEchoesAnim::Clip& clip, float time) const;
  // Turns each bone by the additive clip's rotation at `time`, scaled by `weight`.
  void ApplyAdditive(const PortEchoesAnim::Clip& clip, float time, float weight,
                     PortEchoesAnim::PrimePose& pose) const;

  std::shared_ptr< const Library > mLibrary;
  PortEchoesProbe::Skeleton mPrime;
  std::vector< Moving > mSets[4]; // forward, backward, left, right; slowest first
  const PortEchoesAnim::Clip* mIdle = nullptr;
  const PortEchoesAnim::Clip* mAir = nullptr;
  const PortEchoesAnim::Clip* mLand = nullptr;
  int mRootBone = -1;                       // in mPrime
  std::map< uint32_t, int > mPrimeOfEchoes; // Echoes bone id -> Prime bone index

  const PortEchoesAnim::Clip* mAim[4] = {};   // up, down, left, right
  float mAimTime[4] = {};                     // the key that holds the full aim
  float mAimRange[4] = {};                    // radians at full weight
  const PortEchoesAnim::Clip* mShots[3] = {}; // light, medium, heavy
  float mShotTime = -1.f;
  int mShotStrength = 0;
  float mAimPitch = 0.f; // smoothed
  float mAimYaw = 0.f;

  std::vector< int > mLegBones; // in mPrime
  std::map< const PortEchoesAnim::Clip*, std::vector< PortEchoesAnim::Quat > > mMeanLegs;
  float mLastStride = 1.f;

  const PortEchoesAnim::Clip* mTurn[2] = {}; // left, right
  float mTurnSpeed[2] = {};                  // radians a second as animated
  float mTurnRate = 0.f;                     // smoothed input
  float mTurnTime = 0.f;
  const PortEchoesAnim::Clip* mTakeoff[2] = {}; // from standing, from running
  const PortEchoesAnim::Clip* mDoubleJump = nullptr;
  const PortEchoesAnim::Clip* mDashInto[2] = {}; // left, right
  const PortEchoesAnim::Clip* mDashLoop[2] = {};
  const PortEchoesAnim::Clip* mDashLand[2] = {};
  const PortEchoesAnim::Clip* mMorph[2] = {}; // into the ball, out of it
  const PortEchoesAnim::Clip* mTakeoffClip = nullptr;
  float mTakeoffTime = -1.f;
  float mDoubleTime = -1.f;
  bool mDashAir = false;
  int mDashSide = 0;
  const PortEchoesAnim::Clip* mLandClip = nullptr;
  int mMorphState = 0;
  float mMorphTime = 0.f;

  float mForward = 0.f; // smoothed input, skeleton units a second
  float mSide = 0.f;
  float mPhase = 0.f; // shared by the moving clips, 0..1
  float mIdleTime = 0.f;
  float mAirTime = 0.f;
  float mAirClipTime = 0.f;
  float mAirWeight = 0.f;
  float mLandTime = -1.f; // < 0: not landing
  bool mFresh = true;
  std::vector< Contribution > mLast;
  PortEchoesAnim::PrimePose mPose;
};

} // namespace PortEchoesLocomotion
