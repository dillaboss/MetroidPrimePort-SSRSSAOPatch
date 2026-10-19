#pragma once

#include "port_echoes_anim.h"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Echoes' Samus locomotion played on Prime's Samus in the third-person view: standing, walking,
// jogging and running forwards and backwards, strafing, in the air and landing, blended by the
// player's speed and direction and carried onto Prime's skeleton (port_echoes_anim.h). Pure: no
// engine types, so it can be tested on its own; port_third_person.cpp feeds it the player's
// motion and puts its pose into the body's pose builder.
namespace PortEchoesLocomotion {

// The Echoes animations this plays, by name.
const std::vector< std::string >& ClipNames();

struct Library {
  PortEchoesProbe::Skeleton echoes;
  std::map< std::string, PortEchoesAnim::Clip > clips;
  std::vector< PortEchoesAnim::ConventionScore > ranking; // best first
  std::vector< std::string > missing;                     // names not found or not decoded
};

// Decodes the clips and ranks the ways their keys could apply. False, with `error`, when Samus
// cannot stand and walk with what was found.
bool BuildLibrary(const PortEchoesProbe::Skeleton& echoes,
                  const std::vector< std::pair< std::string, std::vector< uint8_t > > >& data,
                  Library& out, std::string& error);

struct Input {
  float dt = 0.f;
  float forwardSpeed = 0.f; // world units a second, in the body's frame
  float sideSpeed = 0.f;    // to the right
  bool onGround = true;
  float scale = 1.f; // world units per skeleton unit (the body model's scale)
};

class Player {
public:
  void SetLibrary(std::shared_ptr< const Library > library);
  void SetPrime(const PortEchoesProbe::Skeleton& prime);
  // An index into PortEchoesAnim::Conventions().
  void SetConvention(size_t index) { mConvention = index; }
  bool Ready() const { return mLibrary != nullptr && !mPrime.bones.empty(); }
  void Reset();
  // Advances by `in.dt` and returns the pose on Prime's skeleton (indexed like the skeleton given
  // to SetPrime). The root keeps its height but not the clips' travel: the player moves the body.
  const PortEchoesAnim::PrimePose& Update(const Input& in);

  // What the blend is made of, for display: clip name and weight, heaviest first.
  std::vector< std::pair< std::string, float > > Weights() const;

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
  };

  void Classify();
  PortEchoesAnim::PrimePose PoseOf(const PortEchoesAnim::Clip& clip, float time) const;

  std::shared_ptr< const Library > mLibrary;
  PortEchoesProbe::Skeleton mPrime;
  size_t mConvention = 0;
  std::vector< Moving > mSets[4]; // forward, backward, left, right; slowest first
  const PortEchoesAnim::Clip* mIdle = nullptr;
  const PortEchoesAnim::Clip* mAir = nullptr;
  const PortEchoesAnim::Clip* mLand = nullptr;
  int mRootBone = -1; // in mPrime

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
