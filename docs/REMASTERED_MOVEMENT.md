# Remastered movement on the ground

What the port runs on the ground when "Remastered movement" is on
(`remastered_movement`, on by default, Controls > Controller settings > Stick aim,
`MP_REMASTERED_MOVEMENT=<0|1>`; see `docs/SETTINGS.md`) and the player is unmorphed.
Addresses are Remastered 1.0.0 offsets, as elsewhere in these notes.

## What is implemented

The grounded path is `CGameCollision::MoveGroundedRemastered`
(`src/MetroidPrime/CGameCollision.cpp`): the port of
`CPhysicsGroundMoverColliderMP1::MoveObject` 0xf923d8 with its `MoveAlongGround`
0xf92ccc and `SnapToGround` 0xf934ec passes, over **static terrain only**. The
decisions are pure and header-only
(`platform/include/port_remastered_ground_policy.h`), unit-tested by
`tests/port_remastered_ground_policy.cpp`: the time search, the wall and floor
response, the up/down step probe, the snap selection and the retained ground/air
state. Contacts come from the port's own terrain producers, not from a retail
contact list.

Per tick the mover saves the loop-top prediction (the absolute snap target and the
whole motion state, tracked separately), runs up to five `MoveAlongGround` passes
(each with a bisection time search, the wall pass, then the floor pass only when no
wall was accepted), runs the terrain `DoCollisionFailsafe` 0xc1fb88 on every loop
exit, snaps to the selected contact, and then sends the source's post-snap
messages (`kSM_OnFloor` / `kSM_Falling`, the surface material, the floor normal,
the ground-collider material).

The integrator is the **player's own split one** (the local class with vptr
0x20e7340), not `CPhysicsEulerIntegrator`: `Pre` 0xc59814 folds the entry impulse
into the momentum before the move; every `PredictMotion` 0xc23e40 in between is
read-only on the linear state, so the tick moves at the pre-force velocity; and
`Post` 0xc5994c adds `dt * (force + persistentMomentum)` after the loop and the
snap, before the transient clear and the messages. The angular integration is the
port's existing one.

The retained ground/air result — the Remastered persisted physics object state at
`CPhysicsState+0x13c` (read as `obj+0x1224`, written through `SetObjectState(1)`
0xc162fc), one of the fields the movers persist and the one this path consumes — is
tracked in `CPlayer`, so a refused snap resends the previous message exactly as
`PostPhysicsUpdate` 0xc545bc does.

## What is not implemented or verified

- **The retail path is untouched.** With Remastered movement off (`rm0`), in
  classic controls, in morph ball, on a save-state load, or on any tick the mover
  stands aside, the tick runs the original `CGroundMovement` /
  `MoveGroundCollider_New` code. Nothing was changed inside it, and there is no
  switch on this path.
- **Unsupported cases fall back to retail; they are not approximated.** The
  Remastered mover does not run on a tick with dynamic contacts (there is no
  Remastered producer for them), in morph ball, on a save-state load, or when the
  mover gate in `CGameCollision::MovePlayer` is closed. Those ticks run the retail
  mover and mark the retained ground/air result invalid; it stays invalid until a
  verified Remastered airborne tick writes 0 again, so a stale entry state is never
  reused. The dynamic-contact test is the port's own: the mover's query region is
  overlap-tested against the near list with the actor's material list, so a detected
  dynamic contact does use retail — but the source's material-filtered near-list
  coverage, i.e. exactly which dynamic colliders are caught and which pass through,
  is **not** verified.
- **The source loop is exact; the terrain side feeding it is not.** The five-pass
  `MoveObject` loop, its bisection time search, the wall pass, the floor pass (only
  when no wall was accepted) and the snap ladder are a term-for-term port of
  `MoveAlongGround`/`SnapToGround` and are unit-tested. What is **not** verified is
  the terrain tree walk that supplies their contacts: the port builds them through
  its own octree cache (`TerrainContacts`), so the contact *traversal order* and the
  resulting per-contact sequence are the port's, not the source's tree walk — a tick
  with several terrain contacts is not proven to visit them in the source's order.
  Platform riders and the angular integration are likewise still the port's existing
  behaviour.
- **Resting pose.** The Remastered snap rests the box bottom at the selected
  contact height **+ 0.002**, where the retail mover's step-down snap ends at
  contact **+ 0.0005**. The 1.5 mm difference is each mover's own source constant
  and is not tuned away. Because the shipped airborne mover's bisection samples are
  discrete, that higher resting pose can make a jump land **one tick later** than
  with the retail mover at 60/120 Hz.
- **No parity claim.** No Remastered/Eden capture has been compared against this
  path, and there is no per-tick comparison beyond the port's own tests and
  telemetry. Ground replacement being wired is not the same as matching
  Remastered.
