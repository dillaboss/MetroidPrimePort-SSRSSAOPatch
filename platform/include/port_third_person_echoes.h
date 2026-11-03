#pragma once

#include <cstddef>
#include <string>
#include <vector>

class CAnimData;
class CQuaternion;
class CStateManager;
class CVector3f;

// Echoes' Samus animations for the third-person body (port_third_person_body.cpp), from the
// user's own Metroid Prime 2: Echoes image. Prime's cutscene clips have no strafe, walk back,
// side dash or double jump; Echoes' multiplayer Samus has all of them, on a skeleton whose joints
// sit where Prime's do. While this is on and the image has been read, the body's legs and stance
// come from Echoes (port_echoes_locomotion.h): standing, walking, jogging and running in every
// direction, turning on the spot, take-offs, the jump loop, double jumps, side dashes and
// landings, and each shot's recoil. The body's own layers stay on top: the aim (and the arm
// turned to the aim point), the scan, a flinch, the grapple reach.
namespace PortThirdPersonEchoes {

// Settings `third_person_echoes_anims` and `echoes_image`.
bool Enabled();
void SetEnabled(bool on);
std::string ImagePath();
void SetImagePath(const std::string& path);

// Starts reading the animations from the image (on a thread of its own, once per image) when on
// and an image is set; the settings call it so they are ready before play.
void Load();

struct State {
  enum class Phase { None, Loading, Ready, Failed };
  Phase phase = Phase::None;          // None: off, or no image
  std::string message;                // Failed: why
  size_t clips = 0;                   // Ready: animations read
  std::vector< std::string > missing; // Ready: the ones the image lacks
  bool playing = false;               // Ready and posing the body now
};
State GetState();

// Once per tick from the body's update, with the movement it measured: fwd and side the ground
// speeds along the facing and across it (right +), rise the vertical one, in world units a
// second; `scale` the body's model scale. Steps Echoes' blend. False while it is not playing
// (off, not read yet, or the body's skeleton has none of Echoes' bones).
bool Tick(const CStateManager& mgr, const CAnimData& body, float dt, float fwd, float side,
          bool grounded, float rise, float scale);

// The legs' pose from the last Tick, for each segment id below `count`: the rotation and offset
// the pose builder takes. Segments Echoes does not move keep their layout offset and no turn.
// False while not playing.
bool BasePose(const CAnimData& body, CQuaternion* rot, CVector3f* off, int count);

// A shot's recoil, as a turn over each segment's rotation (rot[i] = rot[i] * delta[i]) for the
// body to put over its finished pose. False when none is playing.
bool RecoilDeltas(const CAnimData& body, CQuaternion* delta, int count);

// A shot left the cannon: 0 a beam shot, 1 a missile or combo, 2 a charged shot.
void NoteShot(int strength);

// The body was dropped or rebuilt: start over (the image's animations are kept).
void Reset();

} // namespace PortThirdPersonEchoes
