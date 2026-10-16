"""Production wiring guards for AUDIT-3, alongside executable geometry tests.

Accepts another revision's CGameCollision.cpp so the guards can be checked against
018ee83ce without rebuilding or modifying that revision's worktree.
"""
import pathlib
import sys
import unittest

source = pathlib.Path(sys.argv.pop()).read_text()
air = source.split("void CGameCollision::MoveAirborneStepped(", 1)[1]
air = air.split("#endif", 1)[0]


class Wiring(unittest.TestCase):
    def test_f1_full_target_gate(self):
        self.assertIn("fullTarget = startPos + motion.GetTranslation()", air)
        self.assertIn("PortAirMoverContact::RemainingDisplacement(", air)
        self.assertIn("{startPos.GetX(),startPos.GetY(),startPos.GetZ()}", air)
        self.assertNotIn("{current.GetX(),current.GetY(),current.GetZ()}", air)
        self.assertNotIn("const CVector3f disp = actor.GetTranslation() - entryPos", air)

    def test_f2_no_native_terrain_or_time_override(self):
        self.assertNotIn("DetectStaticCollision_Cached(", air)
        self.assertNotIn("DetectCollisionBoolean_Cached(", air)
        self.assertNotIn("DetectCollision_Cached(", air)
        self.assertNotIn("if (!anyActing)", air)
        self.assertIn("PortAirMoverContact::HasImpacts(terrainContacts.size(), list.GetCount())", air)
        self.assertIn("QueryTerrain(query, tris.data(), tris.size(), produceImpacts)", air)

    def test_f3_dynamic_response_only_accepted(self):
        response = air.split("for (int i = 0; i < acceptedDynamic.GetCount(); ++i)", 1)
        self.assertEqual(len(response), 2, "response must iterate accepted actors, not raw contacts")
        self.assertIn("acceptedDynamic.Add(info, false)", response[0])
        self.assertIn("const CCollisionInfo& info = acceptedDynamic[i]", response[1])
        self.assertNotIn("for (int i = staticCount; i < list.GetCount(); ++i)", air)

    def test_flag100_response_wiring(self):
        self.assertIn("PortAirMoverContact::SkipFlatFloor(", air)
        self.assertIn("PortAirMoverContact::PushDistance(flag100)", air)
        self.assertIn("tc.minVertical, mi, tc.flag100)", air)

    def test_landing_follows_mover_state(self):
        # Terrain events keep the retail convention: matLeft is the touched surface.
        self.assertIn("actor.GetMaterialList(),\n                                     material, primary)", air)
        # 0xc545bc lands on the mover's grounded state; no second 0.85 IsFloor test.
        self.assertNotIn("SendScriptMessages(", air)
        landing = air.split("  if (grounded) {\n    for (int i = 0; i < groundList.GetCount(); ++i)", 1)
        self.assertEqual(len(landing), 2, "grounded must land the player")
        self.assertIn("mgr.DeliverScriptMsg(&actor, kInvalidUniqueId, kSM_OnFloor)",
                      landing[1].split("kSM_Falling", 1)[0])

    def test_platform_rider_on_landing(self):
        # 0xcf1a2c: landing on the rider controller's platform adds the player as a
        # rider (0x1f to the platform) and records the platform (0x1f to the player).
        landing = air.split("  if (grounded) {\n    for (int i = 0; i < groundList.GetCount(); ++i)", 1)
        self.assertEqual(len(landing), 2, "grounded must land the player")
        grounded = landing[1].split("kSM_Falling", 1)[0]
        self.assertIn("TCastToPtr< CScriptPlatform >(mgr.ObjectById(riderId))", grounded)
        self.assertIn("mgr.DeliverScriptMsg(platform, actor.GetUniqueId(), kSM_AddPlatformRider)",
                      grounded)
        self.assertIn("mgr.DeliverScriptMsg(&actor, platform->GetUniqueId(), kSM_AddPlatformRider)",
                      grounded)

    def test_ground_check_reads_dynamic_contact_extent(self):
        # A native box contact's GetPoint() is its clipped polygon's low corner, which
        # sits on the probe's open lower bound; the high corner must be read too.
        check = air.split("struct Ground {", 1)[1].split("struct Stepper {", 1)[0]
        self.assertIn("c.GetExtreme().GetZ()", check)

    def test_remastered_failsafe_replaces_last_free_state(self):
        # 0xf94750 DoCollisionFailsafe 0xc1fa68 behind the gate, then DoTunnelingFailsafe
        # 0xc20214 on every tick; Remastered keeps no last non-colliding state.
        self.assertNotIn("AirMoverIsFree", source)
        self.assertNotIn("SetLastNonCollidingState", air)
        self.assertNotIn("CollisionFailsafe(mgr, cache, actor", air)
        tail = air.split("if (!(fabsf(remaining) < 1e-5f && !stalledFlag)) {", 1)
        self.assertEqual(len(tail), 2, "failsafe gate must stay")
        self.assertIn("const bool resolved = failsafe.Collision();", tail[1])
        tunnel = tail[1].split("failsafe.Tunnel(entryPos, actor.GetTranslation(), tunnelOut)", 1)
        self.assertEqual(len(tunnel), 2, "tunneling must run after the gate")
        self.assertIn("actor.ClearForcesAndTorques();", tunnel[1])
        self.assertIn("PortCollisionFailsafe::DoCollisionFailsafe(", air)
        self.assertIn("PortCollisionFailsafe::DoTunnelingFailsafe(", air)


if __name__ == "__main__":
    unittest.main()
