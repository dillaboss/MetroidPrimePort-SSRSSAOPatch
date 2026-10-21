#pragma once

#include <string>
#include <vector>

class CPlayer;
class CStateManager;
class CTransform4f;

// Experimental third-person view: the world is drawn from behind Samus's shoulder while the game
// keeps playing in first person underneath. Aiming, lock-on, scanning and shots are the
// first-person camera's; the third-person camera only looks at the point that camera aims at, so
// the crosshair stays on what a shot hits. Samus has no walk or aim animations, so her standing
// pose is moved by hand: she leans into turns and strafes, bobs and sways with her stride, dips on
// landing, and bends at the spine towards where she aims. Or, with the user's own Echoes image,
// she plays Echoes' Samus animations instead (port_echoes_locomotion.h).
//
// It applies only while the game's own first-person camera is current: cutscenes, the morph ball,
// the interpolations between them and the free camera all keep their own views.
namespace PortThirdPerson {

// Once per presented frame, before anything reads the view (CStateManager::PreRender).
void UpdateFrame(const CStateManager& mgr);
// Once per simulation tick, after the player's own update (CPlayer::Update).
void Tick(float dt, CPlayer& player, CStateManager& mgr);

// This frame is drawn from behind Samus.
bool Active();
// The view the world is drawn from: the third-person one while Active, else `game`.
CTransform4f View(const CTransform4f& game);
// Samus's body is drawn (Active, and the camera is not pulled into her by a wall).
bool ShowBody();
// The view to draw Samus's body through, so that it leans and bobs without its collision or
// physics moving: `view` with the body's motion for this frame folded in (see
// CPortActorRenderScope). False (and `out` untouched) while there is no motion to apply.
bool BodyView(const CPlayer& player, const CTransform4f& view, CTransform4f& out);

// A shot's start (camera-relative, as CPlayerGun builds it) turned to head for the point under the
// crosshair while the view is behind her: from the shoulder the first-person gun's own direction,
// parallel to the eyes' ray, misses it. Left alone without the view, under a lock (the shot homes
// as usual) or when that point is not ahead of the muzzle.
void AimShot(const CStateManager& mgr, CTransform4f& xf);

// A shot left the arm cannon (CPlayerGun): 0 a normal shot, 1 a missile or combo, 2 a charged
// shot. Echoes' animations play a recoil for it.
void NoteShot(int strength);

// Echoes' animations, read from the user's Echoes image on a thread of their own, once per image.
// The settings start the read while the option is on, so they are ready before play.
void LoadEchoesAnimations(const std::string& path);
struct EchoesAnimationState {
  enum class Phase { None, Loading, Ready, Failed };
  Phase phase = Phase::None; // None: never asked for
  std::string message;       // Failed: why
  size_t clips = 0;          // Ready: how many animations were read
  std::vector< std::string > missing; // Ready: the ones the image did not have
  bool playing = false;               // Ready and posing Samus now
};
EchoesAnimationState EchoesAnimations();

} // namespace PortThirdPerson
