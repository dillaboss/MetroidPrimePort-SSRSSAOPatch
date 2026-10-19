// The Echoes skeleton report's disc reading and background run (see port_echoes_probe.h).
#include "port_echoes_probe.h"

#include "port_debug.h"
#include "port_gci.h"
#include "port_lzo.h"
#include "port_mods.h"
#include "port_paths.h"

#include <aurora/dvd.h>
#include <zlib.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace fs = std::filesystem;

namespace PortEchoesProbe {
namespace {

constexpr uint32_t kANCS = 0x414E4353;
constexpr uint32_t kCINF = 0x43494E46;

std::mutex sMutex;
State sState;

void SetMessage(const std::string& message) {
  std::lock_guard lock(sMutex);
  sState.message = message;
}

bool EndsWithPak(const std::string& name) {
  if (name.size() < 4) {
    return false;
  }
  const char* ext = name.c_str() + name.size() - 4;
  return ext[0] == '.' && (ext[1] | 0x20) == 'p' && (ext[2] | 0x20) == 'a' &&
         (ext[3] | 0x20) == 'k';
}

// One PAK on an image: its table, read through a file handle.
struct Pak {
  std::string name;
  void* file = nullptr;
  PortMods::PakTable table;

  size_t Read(uint64_t offset, uint8_t* out, size_t size) const {
    if (aurora_dvd_base_seek(file, int64_t(offset), SEEK_SET) != int64_t(offset)) {
      return 0;
    }
    size_t done = 0;
    while (done < size) {
      const int64_t got = aurora_dvd_base_read(file, out + done, size - done);
      if (got <= 0) {
        break;
      }
      done += size_t(got);
    }
    return done;
  }
};

bool ReadTable(Pak& pak) {
  std::vector< uint8_t > header;
  size_t needed = 0x10000;
  while (needed <= (64u << 20)) {
    header.resize(needed);
    const size_t got = pak.Read(0, header.data(), header.size());
    if (PortMods::ParsePakTable(header.data(), got, pak.table, needed)) {
      return true;
    }
    if (got < header.size() || needed <= header.size()) {
      return false;
    }
  }
  return false;
}

// A resource as the game reads it: Prime's compressed ones are one zlib stream, Echoes' LZO
// segments.
bool ReadResource(const Pak& pak, const PortMods::PakResource& res, Game game,
                  std::vector< uint8_t >& out, std::string& error) {
  std::vector< uint8_t > raw(res.size);
  if (pak.Read(res.offset, raw.data(), raw.size()) != raw.size()) {
    error = "could not read " + PortMods::FourCCString(res.type) + " in " + pak.name;
    return false;
  }
  if (res.compressed == 0) {
    out = std::move(raw);
    return true;
  }
  if (game == Game::Echoes) {
    if (!PortLzo::DecompressSegmented(raw.data(), raw.size(), out)) {
      error = "could not decompress " + PortMods::FourCCString(res.type) + " in " + pak.name;
      return false;
    }
    return true;
  }
  if (raw.size() < 6) {
    error = "a compressed resource in " + pak.name + " is cut short";
    return false;
  }
  out.assign(
      (size_t(raw[0]) << 24) | (size_t(raw[1]) << 16) | (size_t(raw[2]) << 8) | size_t(raw[3]), 0);
  uLongf length = uLongf(out.size());
  if (uncompress(out.data(), &length, raw.data() + 4, uLong(raw.size() - 4)) != Z_OK ||
      length != out.size()) {
    error = "could not decompress " + PortMods::FourCCString(res.type) + " in " + pak.name;
    return false;
  }
  return true;
}

void ScanDisc(const std::string& path, Game game, DiscScan& scan) {
  const char* gameName = game == Game::Echoes ? "Echoes" : "Prime";
  char id[6] = {};
  u8 discNumber = 0;
  u8 version = 0;
  AuroraDiscImage* image = aurora_disc_image_open(path.c_str(), id, &discNumber, &version);
  if (image == nullptr) {
    scan.error = "could not open " + path + " (not a GameCube disc image, or unreadable)";
    return;
  }
  // Declared before the closer, so the PAK handles are still here when it closes them.
  std::vector< Pak > paks;
  struct Closer {
    AuroraDiscImage* image;
    std::vector< Pak >& paks;
    ~Closer() {
      for (Pak& pak : paks) {
        if (pak.file != nullptr) {
          aurora_dvd_base_close(pak.file);
        }
      }
      aurora_disc_image_close(image);
    }
  } closer{image, paks};
  const std::string gameId(id, 6);
  if (game == Game::Echoes && gameId.compare(0, 3, "G2M") != 0) {
    scan.error =
        gameId.compare(0, 3, "R3M") == 0
            ? "this is Metroid Prime Trilogy (" + gameId +
                  "); its Echoes uses different files. A GameCube Echoes image (G2M...) is needed"
            : "this is not Metroid Prime 2: Echoes (game id " + gameId + ")";
    return;
  }
  if (game == Game::Prime && gameId.compare(0, 3, "GM8") != 0) {
    scan.error = "the port's disc is not Metroid Prime (game id " + gameId + ")";
    return;
  }
  scan.id = gameId + " rev " + std::to_string(version);

  std::vector< std::pair< u32, std::string > > files;
  aurora_disc_image_list(
      image,
      [](u32 index, const char* name, u32, void* user) {
        static_cast< std::vector< std::pair< u32, std::string > >* >(user)->emplace_back(index,
                                                                                         name);
      },
      &files);
  for (const auto& [index, name] : files) {
    if (!EndsWithPak(name)) {
      continue;
    }
    Pak pak;
    pak.name = name;
    pak.file = aurora_disc_image_file_open(image, index);
    if (pak.file == nullptr) {
      scan.notes.push_back("could not open " + name);
      continue;
    }
    paks.push_back(std::move(pak));
    SetMessage(std::string("Reading ") + gameName + "'s " + paks.back().name);
    if (!ReadTable(paks.back())) {
      scan.notes.push_back(name + " is not a PAK this tool reads");
      aurora_dvd_base_close(paks.back().file);
      paks.pop_back();
    }
  }
  if (paks.empty()) {
    scan.error = "no PAKs on the disc";
    return;
  }

  // Every character set once (the same one sits in many PAKs), then each Samus's skeleton.
  std::set< uint32_t > seen;
  int failedSets = 0;
  for (const Pak& pak : paks) {
    SetMessage(std::string("Looking for Samus in ") + gameName + "'s " + pak.name);
    for (const PortMods::PakResource& res : pak.table.resources) {
      if (res.type != kANCS || !seen.insert(res.id).second) {
        continue;
      }
      std::vector< uint8_t > data;
      std::string error;
      if (!ReadResource(pak, res, game, data, error)) {
        ++failedSets;
        if (failedSets <= 5) {
          scan.notes.push_back(error);
        }
        continue;
      }
      ++scan.characterSets;
      std::vector< Character > characters;
      const bool read = ParseCharacterSet(data.data(), data.size(), characters, error);
      bool samus = false;
      for (Character& c : characters) {
        if (!IsSamusCharacter(c.name)) {
          continue;
        }
        samus = true;
        Found f;
        f.pak = pak.name;
        f.ancs = res.id;
        f.character = std::move(c);
        scan.samus.push_back(std::move(f));
      }
      if (!error.empty() && (samus || !read)) {
        char idText[16];
        std::snprintf(idText, sizeof(idText), "%08X", res.id);
        scan.notes.push_back(std::string("ANCS ") + idText + " in " + pak.name + ": " + error);
      }
    }
  }
  if (failedSets > 5) {
    scan.notes.push_back(std::to_string(failedSets) + " character sets could not be read in all");
  }
  for (Found& f : scan.samus) {
    f.skeletonError = "skeleton not found on the disc";
    for (const Pak& pak : paks) {
      for (const PortMods::PakResource& res : pak.table.resources) {
        if (res.type != kCINF || res.id != f.character.skeleton) {
          continue;
        }
        std::vector< uint8_t > data;
        std::string error;
        if (!ReadResource(pak, res, game, data, error)) {
          f.skeletonError = error;
        } else if (!ParseSkeleton(data.data(), data.size(), game, f.skeleton, error)) {
          f.skeletonError = error;
        } else {
          f.skeletonError.clear();
        }
        break;
      }
      if (f.skeletonError.empty()) {
        break;
      }
    }
  }
  if (scan.samus.empty()) {
    scan.error =
        "no Samus character found in " + std::to_string(scan.characterSets) + " character sets";
  }
}

std::string PrimeDiscPath() {
  if (const char* saved = PortDebug::DiscPath()) {
    return saved;
  }
  if (const char* env = std::getenv("MP_DISC")) {
    return env;
  }
  return {};
}

// The report's path, or empty with `error` set.
std::string Run(const std::string& echoesPath, std::string& error) {
  DiscScan echoes;
  ScanDisc(echoesPath, Game::Echoes, echoes);
  DiscScan prime;
  const std::string primePath = PrimeDiscPath();
  if (primePath.empty()) {
    prime.error = "the port's disc path is not saved (pick the disc once from the file dialog)";
  } else {
    ScanDisc(primePath, Game::Prime, prime);
  }
  if (echoes.id.empty()) {
    error = "Echoes: " + echoes.error + ".";
    return {};
  }
  SetMessage("Writing the report");
  const fs::path file =
      PortGci::PathFromString(PortPaths::UserFolder()) / "echoes_skeleton_report.txt";
  std::ofstream out(file, std::ios::binary | std::ios::trunc);
  const std::string text = BuildReport(prime, echoes);
  out.write(text.data(), std::streamsize(text.size()));
  if (!out) {
    error = "Could not write " + PortGci::PathString(file) + ".";
    return {};
  }
  return PortGci::PathString(file);
}

} // namespace

bool Start(const std::string& echoesPath) {
  std::lock_guard lock(sMutex);
  if (sState.running) {
    return false;
  }
  sState = {};
  sState.running = true;
  sState.message = "Opening the Echoes image";
  // Detached: a static joinable thread would call std::terminate at exit.
  std::thread([echoesPath] {
    std::string error;
    const std::string report = Run(echoesPath, error);
    std::lock_guard lock(sMutex);
    sState.running = false;
    sState.finished = true;
    sState.ok = !report.empty();
    sState.reportPath = report;
    sState.message = report.empty() ? error : "Report written to " + report;
  }).detach();
  return true;
}

State Status() {
  std::lock_guard lock(sMutex);
  return sState;
}

int RunFromCommandLine(const std::string& echoesPath) {
  std::string error;
  const std::string report = Run(echoesPath, error);
  if (report.empty()) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  std::printf("%s\n", report.c_str());
  return 0;
}

} // namespace PortEchoesProbe
