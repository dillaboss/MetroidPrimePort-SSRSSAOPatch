#ifndef METROID_PRIME_PORT_PORT_CONSOLE_PHYSICS_H
#define METROID_PRIME_PORT_PORT_CONSOLE_PHYSICS_H
#include <cmath>
#include <cstdio>
#include <string>

// JSON numbers for the `physics` object of `status --json` (issue #117 telemetry).
// The game runs with fast-math off for physics, but a component can still be
// non-finite: a velocity is NaN or infinite for a tick when a force blows up or a
// normalise divides by zero. `%.9g` would print `nan`/`inf`, which no JSON parser
// accepts, so a non-finite component is emitted as `null` and a finite one keeps
// the exact `%.9g` text a float round-trips through.
//
// Header-only and free of game types so the console's formatter can be tested
// without the game: tests/port_console_physics.cpp parses what this produces.
namespace PortConsolePhysics {

// One component: `%.9g` of a finite value, `null` otherwise. 16 bytes hold
// `-1.23456789e-38`; the buffer is returned as a string, so no caller-side
// aggregate buffer can truncate it.
inline std::string JsonNumber(float value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast< double >(value));
  return buf;
}

// `[x,y,z]` with each component through JsonNumber.
inline std::string JsonVector(float x, float y, float z) {
  return "[" + JsonNumber(x) + "," + JsonNumber(y) + "," + JsonNumber(z) + "]";
}

} // namespace PortConsolePhysics

#endif // METROID_PRIME_PORT_PORT_CONSOLE_PHYSICS_H
