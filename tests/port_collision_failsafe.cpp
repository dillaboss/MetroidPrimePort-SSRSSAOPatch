// Remastered collision failsafe decisions (platform/port_collision_failsafe.cpp) on
// hand-built contacts: core push selection 0xc1fc54, wrapper 0xc1fa68, tunneling
// 0xc20214, safe offset 0xc217a0, box-box impacts 0xbf61b4, the AABox ray 0xbf7e50 and
// the sphere ray NCollisionRayCast::Sphere 0xbf6ac0 (wrapper 0xc0289c).
#include "port_collision_failsafe.h"

#include <cfloat>
#include <cmath>
#include <cstdio>

using namespace PortCollisionFailsafe;

static int gFailures = 0;

static void Check(bool ok, const char* what) {
  if (!ok) {
    ++gFailures;
    std::printf("FAIL %s\n", what);
  }
}

static bool Near(float a, float b, float e = 1e-5f) { return std::fabs(a - b) <= e; }
static bool Near(const Vec3& a, const Vec3& b, float e = 1e-5f) {
  return Near(a.x, b.x, e) && Near(a.y, b.y, e) && Near(a.z, b.z, e);
}
static Vec3 Mul_(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }

static Impact Terrain(float depth, Vec3 n) {
  Impact i;
  i.depth = depth;
  i.otherNormal = n;
  return i;
}

static void PushSelection() {
  // Within a contact the shallowest impact; across contacts the deepest of those.
  std::vector< Contact > list(2);
  list[0].impacts = {Terrain(.3f, {0, 0, 1}), Terrain(.1f, {1, 0, 0})};
  list[1].impacts = {Terrain(.2f, {0, 1, 0})};
  Vec3 push;
  Check(SelectPush(list, {}, 1.f / 60.f, push), "push: moved");
  Check(Near(push, {0, .2f + 1e-4f, 0}), "push: deepest contact's shallowest impact");

  // Ties keep the first (strict compares both ways).
  list[1].impacts = {Terrain(.1f, {0, -1, 0})};
  SelectPush(list, {}, 1.f / 60.f, push);
  Check(Near(push, {1e-4f + .1f, 0, 0}), "push: tie keeps the earlier contact");

  // Zero depth still pushes by EPS4 (best starts at -FLT_EPSILON).
  list.resize(1);
  list[0].impacts = {Terrain(0.f, {0, 0, 1})};
  SelectPush(list, {}, 1.f / 60.f, push);
  Check(Near(push, {0, 0, 1e-4f}, 1e-9f), "push: zero depth");

  // Character contact: horizontal, away from the other's centre seen from `from`,
  // capped at worldDt * 50.
  Impact c;
  c.depth = 2.f;
  c.otherCharacter = true;
  c.otherCenter = {3, 4, 10};
  list[0].impacts = {c};
  SelectPush(list, {0, 0, 0}, 1.f / 60.f, push);
  const float cap = 50.f / 60.f;
  Check(Near(push, {-.6f * cap, -.8f * cap, 0}), "push: character horizontal capped");
  list[0].impacts[0].depth = .1f;
  SelectPush(list, {0, 0, 0}, 1.f / 60.f, push);
  Check(Near(push, {-.6f * (.1f + 1e-4f), -.8f * (.1f + 1e-4f), 0}), "push: character depth");

  list.clear();
  Check(!SelectPush(list, {}, 1.f / 60.f, push), "push: empty");
}

static void CoreLoop() {
  // A floor plane at z = 0 overlapping a box whose bottom is at pos.z: one push.
  CoreHooks hooks;
  int calls = 0;
  hooks.collide = [&](const Vec3& pos, std::vector< Contact >& out) {
    ++calls;
    if (pos.z >= 0.f) return false;
    out.resize(1);
    out[0].impacts = {Terrain(-pos.z, {0, 0, 1})};
    return true;
  };
  bool tunnelCalled = false;
  hooks.tunnel = [&](const Vec3& from, const Vec3& to, Vec3&) {
    tunnelCalled = true;
    Check(Near(from, {0, 0, -.25f}) && Near(to, {0, 0, 1e-4f}), "core: tunnel from state to out");
    return false;
  };
  Vec3 out;
  Check(Core({0, 0, -.25f}, 1.f / 60.f, hooks, out), "core: moved");
  Check(Near(out, {0, 0, 1e-4f}), "core: depth + EPS4");
  Check(calls == 2 && tunnelCalled, "core: stops on the first free test");

  // Never resolves: five pushes at most.
  calls = 0;
  hooks.collide = [&](const Vec3&, std::vector< Contact >& out) {
    ++calls;
    out.resize(1);
    out[0].impacts = {Terrain(1.f, {1, 0, 0})};
    return true;
  };
  hooks.tunnel = [](const Vec3&, const Vec3&, Vec3&) { return false; };
  Core({}, 1.f / 60.f, hooks, out);
  Check(calls == 5 && Near(out, {5 * (1.f + 1e-4f), 0, 0}, 1e-4f), "core: five iterations");

  // A tunneling revert replaces the result and reports moved.
  calls = 0;
  hooks.collide = [&](const Vec3&, std::vector< Contact >&) { return false; };
  hooks.tunnel = [](const Vec3& from, const Vec3&, Vec3& o) {
    o = from;
    return true;
  };
  Check(Core({1, 2, 3}, 1.f / 60.f, hooks, out) && Near(out, {1, 2, 3}), "core: tunnel result");

  // Separate object-state origin (0xc1fc54 tunnels from the state transform, the pushes
  // reference `from`): the tunnel hook sees state -> pushedOut, the push is still from `from`.
  hooks.collide = [](const Vec3& pos, std::vector< Contact >& list) {
    if (pos.z >= 0.f) return false;
    list.resize(1);
    list[0].impacts = {Terrain(-pos.z, {0, 0, 1})};
    return true;
  };
  Vec3 seenFrom, seenTo;
  hooks.tunnel = [&](const Vec3& f, const Vec3& t, Vec3&) {
    seenFrom = f;
    seenTo = t;
    return false;
  };
  const Vec3 from = {0, 0, -.25f}, state = {5, 6, 7};
  Check(Core(from, state, 1.f / 60.f, hooks, out), "core-state: moved");
  Check(Near(seenFrom, state) && Near(seenTo, {0, 0, 1e-4f}) && Near(out, {0, 0, 1e-4f}),
        "core-state: tunnel state -> pushed-out, push reference from `from`");
  // The default overload is the airborne case: state == from.
  Core(from, 1.f / 60.f, hooks, out);
  Check(Near(seenFrom, from), "core-state: default overload tunnels from `from`");
}

static void Wrapper() {
  const Vec3 cur{1, 1, 1};
  Vec3 out;
  int coreCalls = 0;
  auto core = [&](Vec3 a, Vec3 b, bool ra, bool rb) {
    return [=, &coreCalls](bool failsafe, Vec3& o) {
      ++coreCalls;
      o = failsafe ? b : a;
      return failsafe ? rb : ra;
    };
  };
  auto blocked = [](bool a, bool b) {
    return [=](bool failsafe, const Vec3&) { return failsafe ? b : a; };
  };
  // No failsafe filter: the core decides.
  Check(DoCollisionFailsafe(cur, false, core({2, 2, 2}, {}, true, false), blocked(true, true), out) &&
            Near(out, {2, 2, 2}),
        "wrapper: no F1290, core result");
  Check(!DoCollisionFailsafe(cur, false, core({2, 2, 2}, {}, false, false), blocked(true, true), out) &&
            Near(out, cur),
        "wrapper: no F1290, core failed");
  // First core free: kept.
  coreCalls = 0;
  Check(DoCollisionFailsafe(cur, true, core({2, 2, 2}, {3, 3, 3}, true, true), blocked(false, false), out) &&
            Near(out, {2, 2, 2}) && coreCalls == 1,
        "wrapper: first result free");
  // First core failed: not moved, false.
  Check(!DoCollisionFailsafe(cur, true, core({2, 2, 2}, {3, 3, 3}, false, true), blocked(false, false), out) &&
            Near(out, cur),
        "wrapper: first core failed");
  // First blocked, second free: the failsafe-filter result.
  Check(DoCollisionFailsafe(cur, true, core({2, 2, 2}, {3, 3, 3}, true, true), blocked(true, false), out) &&
            Near(out, {3, 3, 3}),
        "wrapper: failsafe filter result");
  // First blocked, second core failed: false, not moved.
  Check(!DoCollisionFailsafe(cur, true, core({2, 2, 2}, {3, 3, 3}, true, false), blocked(true, false), out) &&
            Near(out, cur),
        "wrapper: second core failed");
  // Both blocked: true, not moved.
  Check(DoCollisionFailsafe(cur, true, core({2, 2, 2}, {3, 3, 3}, true, true), blocked(true, true), out) &&
            Near(out, cur),
        "wrapper: still stuck reports resolved");
}

static void Tunneling() {
  // Player-like box: 1 x 1 x 2.7, position at the feet.
  const Vec3 pos{0, 0, 0};
  const Box bounds{{-.5f, -.5f, 0}, {.5f, .5f, 2.7f}};
  TunnelHooks hooks;
  int rays = 0;
  RayHit hit;
  Vec3 rayStart, rayDir;
  float rayLen = 0.f;
  hooks.ray = [&](const Vec3& s, const Vec3& d, float l) {
    ++rays;
    rayStart = s;
    rayDir = d;
    rayLen = l;
    return hit;
  };
  int safeCalls = 0;
  hooks.safeOffset = [&](const Vec3&, float maxDist, Vec3& o) {
    ++safeCalls;
    Check(Near(maxDist, .5f + FLT_EPSILON), "tunnel: safe radius = min half + eps");
    o = {0, 0, .01f};
    return true;
  };
  Vec3 out;
  // Short move (<= min half extent): never checked.
  Check(!DoTunnelingFailsafe(bounds, pos, {0, 0, 0}, {.5f, 0, 0}, hooks, out) && rays == 0 &&
            Near(out, {.5f, 0, 0}),
        "tunnel: short move");
  // Miss.
  hit = RayHit{};
  Check(!DoTunnelingFailsafe(bounds, pos, {0, 0, 0}, {2, 0, 0}, hooks, out) && rays == 1,
        "tunnel: miss");
  Check(Near(rayStart, {0, 0, 1.35f}) && Near(rayDir, {1, 0, 0}) && Near(rayLen, 2.f),
        "tunnel: ray from the bounds-centre height");
  // Code 1: revert.
  hit.code = kRay_Inside;
  Check(DoTunnelingFailsafe(bounds, pos, {0, 0, 0}, {2, 0, 0}, hooks, out) && Near(out, {0, 0, 0}),
        "tunnel: start inside reverts");
  // Hit closer than the half width: revert.
  hit.code = kRay_Hit;
  hit.t = .49f;
  hit.normal = {-1, 0, 0};
  hit.orientation = 2;
  Check(DoTunnelingFailsafe(bounds, pos, {0, 0, 0}, {2, 0, 0}, hooks, out) && Near(out, {0, 0, 0}),
        "tunnel: near hit reverts");
  // Wall hit: placed half a width short, raised by cy, no safe offset.
  hit.t = 1.5f;
  safeCalls = 0;
  Check(DoTunnelingFailsafe(bounds, pos, {0, 0, 0}, {2, 0, 0}, hooks, out) &&
            Near(out, {1.f, 0, 1.35f}) && safeCalls == 0,
        "tunnel: wall placement");
  // Floor hit: the safe offset is added.
  hit.normal = {0, 0, 1};
  hit.orientation = -1;
  Check(DoTunnelingFailsafe(bounds, pos, {0, 0, 0}, {2, 0, 0}, hooks, out) &&
            Near(out, {1.f, 0, 1.36f}) && safeCalls == 1,
        "tunnel: floor placement + safe offset");
  // Feet below the position shift the result by the same amount.
  const Box low{{-.5f, -.5f, -.2f}, {.5f, .5f, 2.5f}};
  hit.normal = {-1, 0, 0};
  hit.orientation = 2;
  DoTunnelingFailsafe(low, pos, {0, 0, 0}, {2, 0, 0}, hooks, out);
  Check(Near(out, {1.f, 0, 1.15f + .2f}), "tunnel: foot offset");
}

static void Floor() {
  Check(IsFloor(.86f, 2), "is_floor: steep up");
  Check(IsFloor(-1.f, 0), "is_floor: orientation 0");
  Check(IsFloor(.8f, -1), "is_floor: unknown orientation passes 0.7071");
  Check(IsFloor(.8f, 3), "is_floor: orientation 3");
  Check(!IsFloor(.8f, 1) && !IsFloor(.8f, 2), "is_floor: ceiling/wall need 0.85");
  Check(!IsFloor(.7f, -1), "is_floor: below 0.7071");
}

static void SafeOffset() {
  const Box box{{-.5f, -.5f, 0}, {.5f, .5f, 2.7f}};
  SafeHooks hooks;
  int rays = 0;
  hooks.rayBlocked = [&](const Vec3&, const Vec3&, float) {
    ++rays;
    return false;
  };
  Vec3 off;
  hooks.collidesAt = [](const Vec3&) { return false; };
  Check(CalculateOffsetToSafeCollisionPoint(box, {}, .5f, hooks, off) && Near(off, {}),
        "safe: already free");
  // Free only above z >= 0.03: radius order first, then direction order (+Z is #4).
  hooks.collidesAt = [](const Vec3& p) { return p.z < .03f - 1e-6f; };
  Check(CalculateOffsetToSafeCollisionPoint(box, {}, .5f + FLT_EPSILON, hooks, off) &&
            Near(off, {0, 0, .03f}) && rays == 0,
        "safe: smallest radius, first direction");
  // Free only along +Y (#0): wins over every later direction at the same radius.
  hooks.collidesAt = [](const Vec3& p) { return !(p.y > 0.f); };
  CalculateOffsetToSafeCollisionPoint(box, {}, .5f, hooks, off);
  Check(Near(off, {0, .01f, 0}), "safe: direction order");
  // Never free: no valid offset; radii stop below maxDist.
  hooks.collidesAt = [](const Vec3&) { return true; };
  Check(!CalculateOffsetToSafeCollisionPoint(box, {}, .5f, hooks, off), "safe: none");
  // Rays only from the min half extent on.
  rays = 0;
  CalculateOffsetToSafeCollisionPoint(box, {}, .8f, hooks, off);
  Check(rays == 26, "safe: ray branch at d >= min half (only 0.705)");
  // Direction table: unit vectors, port = (-remX, remZ, remY) of the SPEC order.
  for (const Vec3& d : kSafeDirections) {
    Check(Near(d.x * d.x + d.y * d.y + d.z * d.z, 1.f, 1e-5f), "safe: unit directions");
  }
  Check(Near(kSafeDirections[18], {-.57735026f, .57735026f, .57735026f}), "safe: dir 18");
  Check(kSafeRadii[0] == .01f && kSafeRadii[20] == 8.f, "safe: radii");
}

static void BoxBox() {
  std::vector< Impact > imp;
  // Self sinks 0.1 into the top of a wide slab: smallest impact pushes up.
  const Box slab{{-5, -5, -1}, {5, 5, 0}};
  Check(BoxBoxImpacts({{-.5f, -.5f, -.1f}, {.5f, .5f, 2.6f}}, slab, imp), "boxbox: hit");
  float best = FLT_MAX;
  Vec3 n;
  for (const Impact& i : imp) {
    if (i.depth < best) {
      best = i.depth;
      n = i.otherNormal;
    }
  }
  Check(Near(best, .1f) && Near(n, {0, 0, 1}), "boxbox: vertical push out of the top");
  // Side overlap: +X push from a box on the -X side.
  imp.clear();
  BoxBoxImpacts({{.8f, -.5f, 0}, {1.8f, .5f, 2.7f}}, {{-1, -1, -1}, {1, 1, 5}}, imp);
  Check(imp.size() == 1 && Near(imp[0].depth, .2f) && Near(imp[0].otherNormal, {1, 0, 0}),
        "boxbox: side impact");
  // Self inside other on every axis: fallback picks the smallest difference.
  imp.clear();
  BoxBoxImpacts({{-.5f, -.5f, 0}, {.5f, .5f, 1}}, {{-3, -3, -.25f}, {3, 3, 4}}, imp);
  Check(imp.size() == 1 && Near(imp[0].depth, 1.25f) && Near(imp[0].otherNormal, {0, 0, -1}),
        "boxbox: contained fallback");
  // Touching: no positive depth, no contact.
  imp.clear();
  Check(!BoxBoxImpacts({{1, 0, 0}, {2, 1, 1}}, {{0, 0, 0}, {1, 1, 1}}, imp), "boxbox: touching");
  // Apart.
  Check(!BoxBoxImpacts({{2, 0, 0}, {3, 1, 1}}, {{0, 0, 0}, {1, 1, 1}}, imp), "boxbox: apart");
}

static void Ray() {
  const Box box{{1, -1, -1}, {3, 1, 1}};
  RayHit r = RayAABox(box, {0, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Hit && Near(r.t, 1.f) && Near(r.normal, {-1, 0, 0}), "ray: hit");
  r = RayAABox(box, {0, 0, 0}, {1, 0, 0}, .5f);
  Check(r.code == kRay_Miss, "ray: too short");
  r = RayAABox(box, {2, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Inside && r.t == 0.f, "ray: start inside");
  r = RayAABox(box, {0, 2, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "ray: parallel outside");
  r = RayAABox(box, {2, 0, 5}, {0, 0, -1}, 10.f);
  Check(r.code == kRay_Hit && Near(r.t, 4.f) && Near(r.normal, {0, 0, 1}), "ray: down onto top");
  r = RayAABox(box, {5, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "ray: behind");
}

// ---------------------------------------------------------------------------
// CCollidableSphere::AABox_Sphere 0xc02d70 (dispatcher 0xc02cec). Box-sphere pairs
// below are built in PORT axes; the routine converts to Remastered's Y-up frame, so
// every expected normal is stated in port axes with the frame mapping in mind.
// ---------------------------------------------------------------------------

// Player-like box: 1 x 1 x 2.7, position at the feet.
static const Box kPlayerBox{{-.5f, -.5f, 0}, {.5f, .5f, 2.7f}};

static bool OneSphere(const Box& self, Vec3 center, float radius, Impact& out) {
  std::vector< Impact > v;
  if (!BoxSphereImpacts(self, center, radius, v) || v.size() != 1) return false;
  out = v[0];
  return true;
}

static void BoxSphereFaces() {
  // Six exterior face contacts, gap g = 0.25 and r = 1: the sphere half's normal is
  // radial toward the box, i.e. the port axis pointing from the sphere at the box.
  const float g = .25f, r = 1.f;
  const struct { Vec3 c; Vec3 n; } cases[6] = {
      {{.5f + g, 0, 1.35f}, {-1, 0, 0}},  // +X port face is the -X Rem face
      {{-.5f - g, 0, 1.35f}, {1, 0, 0}},  // -X port face is the +X Rem face
      {{0, .5f + g, 1.35f}, {0, -1, 0}},  // port +Y is Rem +Z
      {{0, -.5f - g, 1.35f}, {0, 1, 0}},
      {{0, 0, 2.7f + g}, {0, 0, -1}},  // port +Z is Rem +Y
      {{0, 0, -g}, {0, 0, 1}},
  };
  const char* names[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  for (int i = 0; i < 6; ++i) {
    Impact im;
    char what[64];
    std::snprintf(what, sizeof what, "boxsphere: face %s hit", names[i]);
    Check(OneSphere(kPlayerBox, cases[i].c, r, im), what);
    std::snprintf(what, sizeof what, "boxsphere: face %s radial normal", names[i]);
    Check(Near(im.otherNormal, cases[i].n, 1e-5f), what);
    std::snprintf(what, sizeof what, "boxsphere: face %s depth r-g", names[i]);
    Check(Near(im.depth, r - g, 1e-5f), what);
    Check(Near(im.otherNormal.x * im.otherNormal.x + im.otherNormal.y * im.otherNormal.y +
                   im.otherNormal.z * im.otherNormal.z,
               1.f, 1e-5f),
          "boxsphere: unit normal");
    Check(!im.otherCharacter && Near(im.otherCenter, {}), "boxsphere: flags left to the caller");
  }
  // The exterior distance is the ARM estimate refined ONCE (0xc032f0-0xc03330), not a
  // correctly-rounded root: for d2 = 0.25^2 it reads 0.249998569, so the depth is
  // 0.750001431 and not 0.75. A two-refinement or sqrt-based distance would give
  // exactly 0.75 here, which is what the second check excludes.
  Impact im;
  Check(OneSphere(kPlayerBox, {.75f, 0, 1.35f}, r, im), "boxsphere: one-refinement hit");
  Check(Near(im.depth, 1.f - 0.249998569f, 5e-8f), "boxsphere: exterior depth, one refinement");
  Check(!Near(im.depth, .75f, 1e-7f), "boxsphere: exterior depth is not the exact distance");
  // Appends like BoxBoxImpacts and leaves earlier impacts alone.
  std::vector< Impact > v;
  v.push_back(Terrain(.5f, {0, 1, 0}));
  Check(BoxSphereImpacts(kPlayerBox, {.75f, 0, 1.35f}, r, v) && v.size() == 2 &&
            Near(v[0].depth, .5f) && Near(v[0].otherNormal, {0, 1, 0}),
        "boxsphere: appends one impact");
}

static void BoxSphereEdgeCorner() {
  // Edge: outside the port +X and +Y faces. The sphere normal is radial, not a box
  // axis, and the depth is r - sqrt(2 * 0.25^2).
  Impact im;
  Check(OneSphere(kPlayerBox, {.75f, .75f, 1.35f}, 1.f, im), "boxsphere: edge hit");
  Check(Near(im.otherNormal, {-0.707106829f, -0.707106829f, 0.f}, 1e-5f),
        "boxsphere: edge radial normal");
  Check(std::fabs(im.otherNormal.x) > .5f && std::fabs(im.otherNormal.y) > .5f &&
            im.otherNormal.z == 0.f,
        "boxsphere: edge normal is not a box axis");
  // d2 = 0.125: the one-refinement distance is 0.353549033, the exact one 0.353553391.
  Check(Near(im.depth, 1.f - 0.353549033f, 5e-8f), "boxsphere: edge depth, one refinement");
  Check(!Near(im.depth, 1.f - .353553391f, 1e-7f),
        "boxsphere: edge depth is not the exact distance");

  // Corner: outside three faces of a cube ({-1..1}^3), gaps 0.5, r = 2. All three
  // components are radial (Rem -X, -Y, -Z map to port +X, -Z, -Y).
  const Box cube{{-1, -1, -1}, {1, 1, 1}};
  Check(OneSphere(cube, {1.5f, 1.5f, 1.5f}, 2.f, im), "boxsphere: corner hit");
  Check(Near(im.otherNormal, {-0.577350259f, -0.577350259f, -0.577350259f}, 1e-5f),
        "boxsphere: corner radial normal");
  // d2 = 0.75: one refinement 0.866020024 against 0.866025404 exact.
  Check(Near(im.depth, 2.f - 0.866020024f, 5e-8f), "boxsphere: corner depth, one refinement");
  Check(!Near(im.depth, 2.f - .866025404f, 1e-6f),
        "boxsphere: corner depth is not the exact distance");
  // The edge normal's bits: two refinements on the Rem vector (0.25, 0, -0.25) give
  // 0.707106829 while a sqrt normalization gives 0.707106769, so this pins the
  // CUnitVector3f path (0x71164 -> 0x6b6b8) rather than an invented unit vector.
  Check(OneSphere(kPlayerBox, {.75f, .75f, 1.35f}, 1.f, im) && im.otherNormal.x == -0.707106829f &&
            im.otherNormal.y == -0.707106829f,
        "boxsphere: edge normal uses the source normalization");

  // Tangency and misses: d2 >= r*r is rejected, exact tangency included.
  std::vector< Impact > v;
  Check(!BoxSphereImpacts(kPlayerBox, {.75f, 0, 1.35f}, .25f, v) && v.empty(),
        "boxsphere: exact tangency is a miss");
  Check(!BoxSphereImpacts(kPlayerBox, {.75f, 0, 1.35f}, .2f, v) && v.empty(),
        "boxsphere: r below the gap is a miss");
  Check(!BoxSphereImpacts(kPlayerBox, {5.f, 0, 1.35f}, 1.f, v) && v.empty(),
        "boxsphere: far miss");
  Check(BoxSphereImpacts(kPlayerBox, {.75f, 0, 1.35f}, .250001f, v) && v.size() == 1 &&
            Near(v[0].depth, .250001f - .249998569f, 5e-8f),
        "boxsphere: a hair more than the gap overlaps");
  v.clear();
}

static void BoxSphereInterior() {
  Impact im;
  // Interior, off centre: nearest signed face distance in Rem order. 0.2 below the
  // port +Z (Rem +Y) face while every other face is at least 0.5 away, so Rem +Y wins.
  Check(OneSphere(kPlayerBox, {0, 0, 2.5f}, 1.f, im), "boxsphere: interior hit");
  Check(Near(im.otherNormal, {0, 0, -1}), "boxsphere: interior normal is -nearest face");
  Check(Near(im.depth, 1.f + (2.7f - 2.5f), 1e-6f), "boxsphere: interior depth r + D");
  // Exact face: the centre on the port +X face (the Rem -X face), D = 0, depth exactly r
  // and the returned normal is +X in Rem, i.e. -X in port axes.
  Check(OneSphere(kPlayerBox, {.5f, 0, 1.35f}, 1.f, im), "boxsphere: centre on a face");
  Check(Near(im.depth, 1.f, 1e-7f) && Near(im.otherNormal, {-1, 0, 0}),
        "boxsphere: on-face depth r and the port -X normal");

  // Coincident centre: ties keep the Rem order -X, -Y, -Z, +X, +Y, +Z, so what comes
  // back is the negative of the winning Rem face, mapped into port axes.
  const Box cube{{-1, -1, -1}, {1, 1, 1}};
  Check(OneSphere(cube, {}, 2.f, im), "boxsphere: coincident hit");
  // All six faces are 1 away: -X keeps the tie against -Y and every later face, so the
  // winner is Rem -X (port +X) and the returned normal is its negative in port axes.
  Check(Near(im.otherNormal, {-1, 0, 0}) && Near(im.depth, 3.f), "boxsphere: tie keeps Rem -X");
  const Box wide{{-2, -1, -1}, {2, 1, 1}};
  Check(OneSphere(wide, {}, 2.f, im), "boxsphere: tie hit 2");
  Check(Near(im.otherNormal, {0, 0, 1}) && Near(im.depth, 3.f),
        "boxsphere: tie keeps Rem -Y (port +Z)");
  const Box slab{{-2, -.5f, -2}, {2, .5f, 2}};
  Check(OneSphere(slab, {}, 2.f, im), "boxsphere: tie hit 3");
  Check(Near(im.otherNormal, {0, 1, 0}) && Near(im.depth, 2.5f),
        "boxsphere: tie keeps Rem -Z (port +Y)");

  // Near/interior is |d2| < 1e-5 and is NOT an inside test: a centre just outside two
  // faces (d2 = 8e-6) still takes the face branch, whose normal is a pure axis where
  // the exterior branch would give a 45 degree radial normal.
  Check(OneSphere(kPlayerBox, {.502f, .502f, 1.35f}, 1.f, im), "boxsphere: near-outside hit");
  Check(Near(im.otherNormal, {-1, 0, 0}, 1e-6f), "boxsphere: below 1e-5 uses the face normal");
  Check(Near(im.depth, 1.f - .002f, 1e-5f), "boxsphere: below 1e-5 depth r - gap");
  // Just above the threshold (d2 = 1.8e-5) the exterior branch runs: radial normal,
  // depth r - d2*rsqrt_est(d2) with the one-step refinement.
  Check(OneSphere(kPlayerBox, {.503f, .503f, 1.35f}, 1.f, im), "boxsphere: above-threshold hit");
  Check(Near(im.otherNormal, {-0.707106829f, -0.707106829f, 0.f}, 1e-5f),
        "boxsphere: above 1e-5 uses the radial normal");
  Check(Near(im.depth, 1.f - 0.0042426406f, 1e-6f), "boxsphere: above 1e-5 depth r - L");
}

static void BoxSpherePush() {
  // The sphere contact feeds the failsafe core like any other: one contact with the
  // sphere's depth and normal, then SelectPush 0xc1fc54 decides the translation.
  std::vector< Impact > v;
  Check(BoxSphereImpacts(kPlayerBox, {.75f, 0, 1.35f}, 1.f, v) && v.size() == 1,
        "bs-push: producer");
  const Impact& i = v[0];
  Vec3 push;
  std::vector< Contact > list(1);
  list[0].impacts = v;
  Check(SelectPush(list, {}, 1.f / 60.f, push) && Near(push, Mul_(i.otherNormal, i.depth + 1e-4f)),
        "bs-push: Solid pushes along the sphere normal by depth + eps");
  // Character / CollisionActor (0x41 flags): horizontal, away from the sphere centre
  // seen from `from`, capped at worldDt * 50.
  list[0].impacts[0].otherCharacter = true;
  list[0].impacts[0].otherCenter = {.75f, 0, 1.35f};
  Check(SelectPush(list, {}, 1.f / 60.f, push) && Near(push, {-i.depth - 1e-4f, 0, 0}, 1e-6f),
        "bs-push: Character pushes horizontally by depth + eps");
  // A deeper overlap hits the worldDt * 50 cap (0.8333 at 60 Hz).
  std::vector< Impact > deep;
  Check(BoxSphereImpacts(kPlayerBox, {.75f, 0, 1.35f}, 2.f, deep) && deep.size() == 1,
        "bs-push: deep producer");
  std::vector< Contact > deepList(1);
  deepList[0].impacts = deep;
  deepList[0].impacts[0].otherCharacter = true;
  deepList[0].impacts[0].otherCenter = {.75f, 0, 1.35f};
  Check(deep[0].depth > 50.f / 60.f, "bs-push: deep overlap exceeds the cap");
  Check(SelectPush(deepList, {}, 1.f / 60.f, push) && Near(push, {-50.f / 60.f, 0, 0}),
        "bs-push: Character push is capped at worldDt * 50");
  // Centre directly above `from`: no horizontal direction, zero push, still moved.
  Check(SelectPush(deepList, {.75f, 0, 0}, 1.f / 60.f, push) && Near(push, {}),
        "bs-push: Character with no horizontal offset pushes nowhere");
  // The core loop (0xc1fc54) must consume the real contact and stop on the first free
  // test, exactly as it does for terrain and box actors.
  CoreHooks hooks;
  int calls = 0;
  hooks.collide = [&](const Vec3&, std::vector< Contact >& out) {
    if (++calls > 1) return false;
    out = deepList;
    return true;
  };
  bool tunneled = false;
  hooks.tunnel = [&](const Vec3& from, const Vec3&, Vec3& out) {
    tunneled = true;
    out = from;
    return false;
  };
  Vec3 out2;
  Check(Core({}, 1.f / 60.f, hooks, out2) && Near(out2, {-50.f / 60.f, 0, 0}) && calls == 2 &&
            tunneled,
        "bs-push: core consumes the sphere contact");

  // First pass versus fallback: the material filter (0x1270) sees the sphere, the
  // failsafe filter (0x1290) excludes Character/CollisionActor, so a position that is
  // still blocked there cannot be resolved by the second pass.
  const Vec3 cur{1, 1, 1};
  const Vec3 first = {2, 2, 2}, second = {3, 3, 3};
  Vec3 out;
  int pass[2] = {0, 0};
  auto core = [&](bool failsafe, Vec3& o) {
    ++pass[failsafe ? 1 : 0];
    o = failsafe ? second : first;
    return !failsafe; // the sphere is not in the 0x1290 filter
  };
  auto blocked = [](bool failsafe, const Vec3&) { return !failsafe; };
  Check(!DoCollisionFailsafe(cur, true, core, blocked, out) && Near(out, cur) && pass[0] == 1 &&
            pass[1] == 1,
        "bs-push: Character sphere is a first-pass contact only");
  auto core2 = [&](bool failsafe, Vec3& o) {
    ++pass[failsafe ? 1 : 0];
    o = failsafe ? second : first;
    return true;
  };
  Check(DoCollisionFailsafe(cur, true, core2, blocked, out) && Near(out, second),
        "bs-push: fallback-safe contact uses the failsafe-filter result");
}

static void SphereRay() {
  // Strictly inside: code 1, t = 0 and the wrapper's untouched skUpVector normal,
  // which is port +Z (Remastered's (0,1,0) under ToRem/FromRem).
  RayHit r = RaySphere({0, 0, 0}, 1.f, {0, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Inside && r.t == 0.f && Near(r.normal, {0, 0, 1}),
        "sphereray: from the centre");
  r = RaySphere({0, 0, 0}, 1.f, {.25f, 0, .25f}, {0, 1, 0}, 5.f);
  Check(r.code == kRay_Inside && r.t == 0.f && Near(r.normal, {0, 0, 1}),
        "sphereray: inside, off centre");
  // A translated centre is measured from the ray start, and the inside arm never
  // reads `length`.
  r = RaySphere({0, 0, 10}, 2.f, {0, 0, 9}, {0, 0, 1}, 5.f);
  Check(r.code == kRay_Inside && r.t == 0.f, "sphereray: inside a translated sphere");
  r = RaySphere({0, 0, 0}, 1.f, {.5f, 0, 0}, {1, 0, 0}, 0.f);
  Check(r.code == kRay_Inside && r.t == 0.f, "sphereray: inside with zero length");

  // The boundary is NOT inside (r2 <= d2): heading in reports the entry at t = 0,
  // heading out misses (B < 0).
  r = RaySphere({0, 0, 0}, 1.f, {1, 0, 0}, {-1, 0, 0}, 5.f);
  Check(r.code == kRay_Hit && Near(r.t, 0.f, 1e-6f) && Near(r.normal, {1, 0, 0}),
        "sphereray: on the boundary heading in");
  r = RaySphere({0, 0, 0}, 1.f, {1, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "sphereray: on the boundary heading out");

  // A straight entry along port +X: the entry point is the near side (port x = -1)
  // and the normal is radial, point - centre.
  r = RaySphere({0, 0, 0}, 1.f, {-2, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Hit && Near(r.t, 1.f, 1e-5f) && Near(r.normal, {-1, 0, 0}, 1e-5f),
        "sphereray: entry along an axis");
  // Translated centre, down port +Z: t = 10 - 2 at (0,0,8), normal -Z.
  r = RaySphere({0, 0, 10}, 2.f, {0, 0, 0}, {0, 0, 1}, 20.f);
  Check(r.code == kRay_Hit && Near(r.t, 8.f, 1e-5f) && Near(r.normal, {0, 0, -1}, 1e-5f),
        "sphereray: entry into a translated sphere");
  // Off centre: the chord sits at |y| = 0.5, so t = 2 - sqrt(0.75) and the normal is
  // the radial (sqrt(0.75), 0.5, 0) in Remastered, i.e. (-0.8660254, 0.5, 0) in port.
  r = RaySphere({0, 0, 0}, 1.f, {-2, .5f, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Hit && Near(r.t, 1.1339746f, 1e-5f), "sphereray: off-centre entry t");
  Check(Near(r.normal, {-0.8660254f, .5f, 0.f}, 1e-5f), "sphereray: off-centre radial normal");
  Check(Near(r.normal.x * r.normal.x + r.normal.y * r.normal.y + r.normal.z * r.normal.z, 1.f,
             1e-5f),
        "sphereray: unit normal");

  // The entry distance uses the source's root (CMath::SqrtF 0x65f70: the ARM
  // reciprocal-sqrt estimate refined TWICE, times the value), not a correctly rounded
  // sqrt. r = 1.0150001 (0x3f81eb86) is a witness: its entry t reads 0.984999776
  // (0x3f7c28f2) where a host root of the same discriminant gives 0.984999895
  // (0x3f7c28f4), so this pins the emulation rather than a plain sqrt.
  const float witnessR = 1.0150001f;
  r = RaySphere({0, 0, 0}, witnessR, {-2, 0, 0}, {1, 0, 0}, 10.f);
  const float hostRoot = 2.f - std::sqrt(witnessR * witnessR);
  Check(r.code == kRay_Hit && r.t == 0.984999776f, "sphereray: source sqrt of the discriminant");
  Check(r.t != hostRoot, "sphereray: not a correctly rounded host sqrt");

  // Misses. `t < length` is strict, so the endpoint itself misses.
  r = RaySphere({0, 0, 0}, 1.f, {-2, 0, 0}, {1, 0, 0}, 1.f);
  Check(r.code == kRay_Miss, "sphereray: endpoint is a miss");
  r = RaySphere({0, 0, 0}, 1.f, {-2, 0, 0}, {1, 0, 0}, 1.0001f);
  Check(r.code == kRay_Hit && Near(r.t, 1.f, 1e-5f), "sphereray: just past the endpoint");
  r = RaySphere({0, 0, 0}, 1.f, {-2, 0, 0}, {1, 0, 0}, .5f);
  Check(r.code == kRay_Miss, "sphereray: too short");
  r = RaySphere({0, 0, 0}, 1.f, {-2, 0, 0}, {1, 0, 0}, 0.f);
  Check(r.code == kRay_Miss, "sphereray: zero length");
  r = RaySphere({0, 0, 0}, 1.f, {-2, 0, 0}, {-1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "sphereray: pointing away");
  r = RaySphere({0, 0, 0}, 1.f, {-2, 1.5f, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "sphereray: past the side");
  // Exact tangency: disc == 0 is excluded (0.0 < disc is strict).
  r = RaySphere({0, 0, 0}, 1.f, {-2, 1.f, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "sphereray: tangent");
  // A degenerate zero radius is never strictly inside and never tangent-positive.
  r = RaySphere({0, 0, 0}, 0.f, {0, 0, 0}, {1, 0, 0}, 5.f);
  Check(r.code == kRay_Miss, "sphereray: zero radius at the centre");
}

int main() {
  PushSelection();
  CoreLoop();
  Wrapper();
  Tunneling();
  Floor();
  SafeOffset();
  BoxBox();
  Ray();
  SphereRay();
  BoxSphereFaces();
  BoxSphereEdgeCorner();
  BoxSphereInterior();
  BoxSpherePush();
  if (gFailures != 0) {
    std::printf("%d failure(s)\n", gFailures);
    return 1;
  }
  std::printf("port_collision_failsafe: ok\n");
  return 0;
}
