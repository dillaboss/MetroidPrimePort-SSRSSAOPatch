#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// Compares Samus's skeleton in Metroid Prime with the one in Metroid Prime 2: Echoes, from the
// user's own discs, and lists the animations Echoes has for her: the first step towards playing
// Echoes' multiplayer animations on Prime's body in the third-person view. Writes a text report
// of names and numbers only (bone and animation names, positions, rotations); no game data is
// copied out.
namespace PortEchoesProbe {

enum class Game { Prime, Echoes };

struct Bone {
  uint32_t id = 0;
  uint32_t parent = 0;
  float position[3] = {};    // skeleton space
  float rotation[4] = {1.f}; // w, x, y, z; Echoes only (Prime's skeletons have none)
  float localRotation[4] = {1.f};
  std::string name;
};

struct Skeleton {
  std::vector< Bone > bones;
  const Bone* Find(uint32_t id) const;
  const Bone* Find(const std::string& name) const;
};

// A CINF. Echoes' bones carry two bind rotations that Prime's don't.
bool ParseSkeleton(const uint8_t* data, size_t size, Game game, Skeleton& out, std::string& error);

struct Character {
  uint32_t id = 0;
  uint16_t version = 0; // 5 or 6 in Prime, 10 in Echoes
  std::string name;
  uint32_t model = 0;
  uint32_t skin = 0;
  uint32_t skeleton = 0;
  std::vector< std::pair< uint32_t, std::string > > animations; // id, name
};

// An ANCS's characters. Their animation names come early in each; reaching the next character
// needs the whole entry read, so a set that can't be read to the end still returns the characters
// before the failure (true with `error` set when at least one was read).
// `end`, when given, is set to where the characters end (the animation set starts there) after a
// whole read, and to 0 otherwise.
bool ParseCharacterSet(const uint8_t* data, size_t size, std::vector< Character >& out,
                       std::string& error, size_t* end = nullptr);

// A Samus character found on a disc, with its skeleton when that could be read.
struct Found {
  std::string pak;
  uint32_t ancs = 0;
  Character character;
  Skeleton skeleton;
  std::string skeletonError; // empty when `skeleton` was read
};

// Whether a character name looks like Samus's body (not her gun or her ball alone).
bool IsSamusCharacter(const std::string& name);

struct DiscScan {
  std::string id;    // game id, empty when the disc was not read
  std::string error; // why it was not, or what went wrong part way
  std::vector< Found > samus;
  int characterSets = 0;
  std::vector< std::string > notes; // characters that could not be read in full, ...
  // Echoes: a few of the picked Samus's animations (name, ANIM bytes; empty when not found) for
  // the decode check, or why they could not be gathered.
  std::vector< std::pair< std::string, std::vector< uint8_t > > > clips;
  std::string clipError;
};

// The Samus the comparison uses: Prime's body ("Samus"), Echoes' with the most animations.
const Found* PickSamus(const DiscScan& scan, Game game);

std::string BuildReport(const DiscScan& prime, const DiscScan& echoes);

// --- The run, on a background thread --------------------------------------------------------

struct State {
  bool running = false;
  bool finished = false;
  bool ok = false;
  std::string message;
  std::string reportPath;
};

// Reads the Echoes image at `path` (and the Prime disc the port plays) and writes the report into
// the user folder. False when a run is already going.
bool Start(const std::string& echoesPath);
State Status();
// The same, blocking, for `--echoes-report <image>`: the report's path on stdout, 0 on success.
int RunFromCommandLine(const std::string& echoesPath);

} // namespace PortEchoesProbe
