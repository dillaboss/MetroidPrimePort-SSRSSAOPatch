#pragma once

#include <string>

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

// A shot left the arm cannon (CPlayerGun): 0 a normal shot, 1 a missile or combo, 2 a charged
// shot. Echoes' animations play a recoil for it.
void NoteShot(int strength);

// Echoes' animations: loading, playing (with how their keys are read and what is blending), or
// why not. Empty while they are off.
std::string EchoesAnimationStatus();

} // namespace PortThirdPerson
