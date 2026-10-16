#include "port_remastered_ground_contacts.h"
#ifdef NDEBUG
#undef NDEBUG // These standalone regression checks must run in release builds.
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>

using namespace PortRemasteredGroundContacts;
namespace {
constexpr AABB Box{{-1.f,-1.f,-1.f},{1.f,1.f,1.f}};
bool Near(float a, float b, float epsilon=2e-6f) { return std::fabs(a-b)<epsilon; }
void Equal(Vec3 a, Vec3 b) { assert(Near(a.x,b.x)); assert(Near(a.y,b.y)); assert(Near(a.z,b.z)); }
Vec3 Add(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 Mul(Vec3 a, float b) { return {a.x*b,a.y*b,a.z*b}; }
Vec3 Cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
std::array<Vec3,3> Face(Vec3 origin, Vec3 n) {
  const Vec3 u=Source::Unit(Cross(n,std::fabs(n.z)<.9f ? Vec3{0,0,1} : Vec3{1,0,0}));
  const Vec3 v=Cross(n,u);
  return {Add(origin,Add(Mul(u,-8),Mul(v,-8))), Add(origin,Add(Mul(u,8),Mul(v,-8))),
          Add(origin,Mul(v,8))};
}
void Invariants(const Result& r) {
  if (!r.hit) return;
  assert(r.pointCount>=2 && r.pointCount<=10);
  assert(r.box.firstPoint==0 && r.box.lastPoint==0);
  assert(r.triangle.firstPoint==1 && r.triangle.lastPoint==r.pointCount-1);
  assert(!r.box.hasSecondary && r.triangle.D==0 && r.triangle.H==0);
  assert(r.provenance.polygonSize>0 && r.provenance.polygonSize<=9);
  if (r.provenance.crossingCount) {
    Equal(r.points[0],r.provenance.lastIntersection);
    assert(r.box.D>=0 && r.box.H>=0);
    for (std::size_t i=1;i<r.pointCount;++i) {
      Equal(r.points[i],r.provenance.polygon[i-1]);
      assert(r.points[i].x>=-1.00001f && r.points[i].x<=1.00001f);
      assert(r.points[i].y>=-1.00001f && r.points[i].y<=1.00001f);
      assert(r.points[i].z>=-1.00001f && r.points[i].z<=1.00001f);
    }
  }
}
void Helpers() {
  assert(!Source::IsNormalizable({0,0,0}));
  assert(!Source::IsNormalizable({1e-15f,0,0}));
  assert(Source::IsNormalizable({1e-14f,0,0}));
  assert(!Source::IsNormalizable({std::nextafter(1e-14f,0.f),0,0}));
  assert(!Source::IsNormalizable({std::numeric_limits<float>::infinity(),0,0}));
  assert(!Source::IsNormalizable({0,std::numeric_limits<float>::quiet_NaN(),0}));
  Equal(Source::Unit({0,0,0}),{});
  Equal(Source::Normalize({3,4,0}),{.6f,.8f,0});
  Equal(Source::Corner(Box,1),{1,-1,-1});
  Equal(Source::Corner(Box,2),{-1,-1,1});
  Equal(Source::Corner(Box,4),{-1,1,-1});
  const Plane plane{{1,0,0},0};
  assert(Source::ClipLineSegment(plane,{-1,0,0},{1,0,0})==.5f);
  assert(Source::ClipLineSegment(plane,{2,0,0},{3,0,0})==0);
  assert(Source::ClipLineSegment(plane,{-2,0,0},{-1,0,0})==1);
  assert(Source::ClipLineSegment(plane,{0,0,0},{0,1,0})==1);
  float d=-99;
  const Vec3 a{-2,0,-2}, b{0,0,2}, c{2,0,-2};
  assert(Source::TriangleNoBackface({0,-1,0},{0,1,0},1,c,b,a,d) && Near(d,1));
  assert(!Source::TriangleNoBackface({0,-1,0},{0,1,0},.9f,c,b,a,d));
  assert(!Source::TriangleNoBackface({0,-1,0},{0,1,0},2,a,b,c,d));
  assert(!Source::TriangleNoBackface({4,-1,0},{0,1,0},2,c,b,a,d));
  assert(!Source::TriangleNoBackface({0,-1,0},{1,0,0},2,c,b,a,d));
  assert(!Source::TriangleNoBackface({0,1,0},{0,1,0},2,c,b,a,d));
  // Infinite-line helper: parallel sweep and parallel axis reject; signs and
  // deliberate lack of plane-intersection division are checked separately.
  assert(!Source::SweptLineLine({0,0,0},{1,0,0},{1,0,0},{0,1,0},{0,0,1},d));
  assert(!Source::SweptLineLine({0,0,0},{1,0,0},{0,1,0},{0,1,0},{1,0,0},d));
  assert(Source::SweptLineLine({0,0,0},{1,0,0},{0,1,0},{0,1,0},{0,0,1},d));
  assert(Near(d,1));
  assert(!Source::SweptLineLine({0,0,0},{1,0,0},{0,1,0},{0,-1,0},{0,0,1},d));
}
void Basic() {
  const Vec3 up{0,1,0};
  const auto flat=Build(Box,Face({0,0,0},up),up);
  Invariants(flat);
  assert(flat.hit && Near(flat.box.D,1) && Near(flat.box.H,1));
  assert(flat.provenance.crossingCount==4 && flat.provenance.polygonSize==4);
  Equal(flat.box.primary,{0,0,1});
  Equal(flat.triangle.primary,up);
  assert(flat.triangle.hasSecondary);
  const auto wall=Build(Box,Face({0,0,0},{1,0,0}),{1,0,0});
  Invariants(wall);
  assert(wall.hit && Near(wall.box.D,1) && Near(wall.box.H,1));
  const Vec3 normal=Source::Unit({0,1,1});
  const auto slope=Build(Box,Face({},normal),normal);
  Invariants(slope);
  assert(slope.hit && Near(slope.box.H,std::sqrt(2.f)));
  assert(slope.box.D>0);
  const std::array<Vec3,3> small={Vec3{-.2f,0,-.2f},Vec3{0,0,.2f},Vec3{.2f,0,-.2f}};
  const auto contained=Build(Box,small,up);
  Invariants(contained);
  assert(contained.hit && contained.provenance.crossingCount==0);
  assert(contained.box.D==-1 && contained.box.H==-1); // Unclamped branch.
  Equal(contained.box.primary,{0,-1,0});
  Equal(contained.points[0],{-1,1,-1});
  assert(!contained.triangle.hasSecondary && contained.pointCount==4);
  for (unsigned i=0;i<3;++i) Equal(contained.points[i+1],small[i]);
  const auto negative=Build(Box,small,{0,-1,0});
  Equal(negative.box.primary,{0,1,0});
  assert(negative.box.D==-1 && negative.box.H==-1);
  const auto tie=Build(Box,small,Source::Unit({1,1,1}));
  Equal(tie.box.primary,{-1,0,0}); // Source table chooses X on three-way tie.
  // A single crossing plane, retain duplicate/on-plane intersections exactly.
  const std::array<Vec3,3> single={Vec3{-.5f,0,-.5f},Vec3{-.5f,0,.5f},Vec3{2,0,0}};
  const auto clipped=Build(Box,single,up);
  Invariants(clipped);
  assert(clipped.hit && clipped.provenance.crossingCount==1);
  assert(clipped.provenance.crossingPlanes[0]==1 && clipped.provenance.polygonSize==4);
  assert(clipped.provenance.secondaryBranch==SecondaryBranch::SingleClipPlane);
  Equal(clipped.box.primary,{1,0,0});
  Equal(clipped.triangle.secondary,{-1,0,0});
  Equal(clipped.provenance.lastIntersection,{1,0,-.2f});
  const auto empty=Build(Box,Face({0,2,0},up),up);
  assert(!empty.hit && empty.provenance.polygonSize==0);
  // All endpoints exactly on a box plane => source rejects (not inclusive SAT).
  assert(!Build(Box,Face({0,1,0},up),up).hit);
  const auto degenerate=Build(Box,{Vec3{},Vec3{},Vec3{}},up);
  assert(degenerate.hit && degenerate.pointCount==4); // No invented area filter.
  assert(!Build(Box,{Vec3{2,0,0},Vec3{2,0,0},Vec3{2,0,0}},up).hit);
}
void SecondaryCases() {
  const Vec3 n=Source::Unit({1,1,1});
  const Vec3 cornerOrigin=Add({-1,-1,-1},Mul(n,.02f));
  const auto corner=Build(Box,Face(cornerOrigin,n),n);
  Invariants(corner);
  assert(corner.hit && corner.provenance.secondaryBranch==SecondaryBranch::ShallowCorner);
  assert(corner.provenance.behindCorners==1 && corner.provenance.shallowCorners==1);
  Equal(corner.triangle.secondary,n);
  const Vec3 edgeN=Source::Unit({1,1,0});
  const auto edge=Build(Box,Face(Add({-1,-1,0},Mul(edgeN,.02f)),edgeN),edgeN);
  Invariants(edge);
  assert(edge.hit && edge.provenance.secondaryBranch==SecondaryBranch::ShallowEdge);
  assert(edge.provenance.behindCorners==5 && edge.provenance.shallowCorners==5);
  Equal(edge.triangle.secondary,edgeN);
  const auto blend=Build(Box,Face(Add({-1,-1,-1},Mul(n,.2f)),n),n);
  Invariants(blend);
  assert(blend.hit && blend.provenance.secondaryBranch==SecondaryBranch::CandidateBlend);
  assert(blend.provenance.candidateCount>=3);
  assert(blend.provenance.candidates[0].point!=blend.provenance.candidates[1].point);
  const auto none=Build(Box,{Vec3{-4,4,-4},Vec3{-4,4,4},Vec3{4,-4,0}},edgeN);
  Invariants(none);
  assert(none.hit && none.provenance.secondaryBranch==SecondaryBranch::NoCandidate);
  assert(!none.triangle.hasSecondary);
  // Shallow-edge rejection is reachable for a very short box edge: source
  // normalizability uses absolute squared length, not a relative angle test.
  const AABB thin{{-1,-1,-1e-15f},{1,1,1e-15f}};
  const auto shortEdge=Build(thin,{Vec3{-2,.02f,0},Vec3{.02f,-2,-.5e-15f},Vec3{.02f,-2,.5e-15f}},edgeN);
  assert(shortEdge.hit);
  assert(shortEdge.provenance.secondaryBranch==SecondaryBranch::UnnormalizableShallowEdge);
  assert(!shortEdge.triangle.hasSecondary);
  // Tiny candidate depths, with six corners behind to bypass shallow edge.
  const AABB positive{{0,0,-1},{1,1,1}};
  const Vec3 reverseN=Mul(edgeN,-1);
  const auto tinyBlend=Build(positive,{Vec3{-1e-15f,1.1e-15f,-.5f},Vec3{1.1e-15f,-1e-15f,-.5f},Vec3{1.1e-15f,-1e-15f,.5f}},reverseN);
  assert(tinyBlend.hit);
  assert(tinyBlend.provenance.secondaryBranch==SecondaryBranch::UnnormalizableCandidateBlend);
  assert(!tinyBlend.triangle.hasSecondary);
}
void Penetration() {
  const Vec3 n=Source::Unit({1,1,0});
  const auto fallback=Build(Box,{Vec3{-2,.1f,0},Vec3{.1f,-2,-.5f},Vec3{.1f,-2,.5f}},n);
  Invariants(fallback);
  const auto& p=fallback.provenance;
  assert(fallback.hit && p.usedEdgeFallback && p.cornerRayHits==0);
  assert(p.polygonRayDistance==0 && p.cornerRayDistance==0);
  unsigned successes=0;
  for (const auto hits:p.fallbackAxisHits) successes |= hits;
  assert(successes!=0);
  // Captured from the original standalone AArch64 routine, not a geometric
  // approximation: this fallback gives D != H (and D > H).
  assert(fallback.box.D==0x1.ee6e54p-4f);
  assert(fallback.box.H==0x1.21a18ap-4f);
  assert(p.finalDistance==fallback.box.D);
  const auto tiny=Build(Box,{Vec3{-2,1e-6f,0},Vec3{1e-6f,-2,-.5f},Vec3{1e-6f,-2,.5f}},n);
  assert(tiny.hit && tiny.provenance.usedEdgeFallback);
  assert(std::fabs(tiny.provenance.finalDistance)<1e-5f);
  assert(tiny.box.D==tiny.box.H && tiny.box.H>0);
  // A corner ray supplies depth absent from all polygon exit rays.
  const auto cornerRay=Build(Box,{Vec3{-4,4,-4},Vec3{-4,4,4},Vec3{4,-4,0}},n);
  assert(cornerRay.provenance.polygonRayDistance==0);
  assert(cornerRay.provenance.cornerRayDistance>1 && cornerRay.provenance.cornerRayHits!=0);
  assert(!cornerRay.provenance.usedEdgeFallback);
}
void DeterministicSweep() {
  std::uint32_t state=9123123;
  auto next=[&]() { state=state*1664525u+1013904223u; return float(int(state>>8)%8193-4096)/512.f; };
  unsigned hits=0;
  for (unsigned i=0;i<10000;++i) {
    const std::array<Vec3,3> t={Vec3{next(),next(),next()},Vec3{next(),next(),next()},Vec3{next(),next(),next()}};
    const Vec3 a{t[1].x-t[0].x,t[1].y-t[0].y,t[1].z-t[0].z};
    const Vec3 b{t[2].x-t[0].x,t[2].y-t[0].y,t[2].z-t[0].z};
    const auto result=Build(Box,t,Source::Unit(Cross(a,b)));
    Invariants(result);
    hits+=result.hit;
  }
  assert(hits>1000 && hits<9000);
}
}
int main() {
  Helpers(); Basic(); SecondaryCases(); Penetration(); DeterministicSweep();
  std::puts("Remastered ground contacts: all checks passed");
  return 0;
}
