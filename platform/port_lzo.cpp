// LZO1X decompression (see port_lzo.h). The decoder follows the reference "safe" decoder's state
// machine: every read of the input and every write or back-reference into the output is checked.
#include "port_lzo.h"

#include <cstring>

namespace PortLzo {

bool Decompress1x(const uint8_t* in, size_t inSize, uint8_t* out, size_t outSize) {
  const uint8_t* ip = in;
  const uint8_t* const ipEnd = in + inSize;
  uint8_t* op = out;
  uint8_t* const opEnd = out + outSize;
  size_t t = 0;
  size_t next = 0;
  // 0: a literal run may follow; 1-3: that many literals followed the last match; 4: a long
  // literal run did.
  size_t state = 0;
  // A match's distance back from `op` (1 = the last byte written).
  size_t dist = 0;

  const auto needIn = [&](size_t n) { return size_t(ipEnd - ip) >= n; };
  const auto needOut = [&](size_t n) { return size_t(opEnd - op) >= n; };
  // A run of zero bytes, then a byte: a long length past `base`.
  const auto longLength = [&](size_t base, size_t& length) {
    size_t zeros = 0;
    while (true) {
      if (!needIn(1)) {
        return false;
      }
      if (*ip != 0) {
        break;
      }
      ++ip;
      if (++zeros > (size_t(-1) / 255) - 2) {
        return false;
      }
    }
    length += zeros * 255 + base + *ip++;
    return true;
  };
  const auto copyLiterals = [&](size_t n) {
    if (!needIn(n) || !needOut(n)) {
      return false;
    }
    std::memcpy(op, ip, n);
    op += n;
    ip += n;
    return true;
  };
  // Distances are checked against what has been written before a pointer is formed from them.
  const auto copyMatch = [&](size_t back, size_t n) {
    if (back == 0 || back > size_t(op - out) || !needOut(n)) {
      return false;
    }
    const uint8_t* from = op - back;
    // Byte by byte: the source may overlap what is being written.
    for (size_t i = 0; i < n; ++i) {
      op[i] = from[i];
    }
    op += n;
    return true;
  };

  if (!needIn(1)) {
    return false;
  }
  bool skipToLiteralRun = false;
  bool skipToMatchNext = false;
  if (*ip > 17) {
    t = size_t(*ip++) - 17;
    if (t < 4) {
      next = t;
      skipToMatchNext = true;
    } else {
      skipToLiteralRun = true;
    }
  }

  while (true) {
    if (skipToMatchNext) {
      skipToMatchNext = false;
      // match_next: the 0-3 literals that follow a match.
      state = next;
      if (!copyLiterals(next)) {
        return false;
      }
      continue;
    }
    if (skipToLiteralRun) {
      skipToLiteralRun = false;
      if (!copyLiterals(t)) {
        return false;
      }
      state = 4;
      continue;
    }

    if (!needIn(1)) {
      return false;
    }
    t = *ip++;
    if (t < 16) {
      if (state == 0) {
        // A literal run.
        if (t == 0 && !longLength(15, t)) {
          return false;
        }
        t += 3;
        if (!copyLiterals(t)) {
          return false;
        }
        state = 4;
        continue;
      }
      if (state != 4) {
        // A 2-byte match within 1 kB, right after 1-3 literals.
        if (!needIn(1)) {
          return false;
        }
        next = t & 3;
        dist = 1 + (t >> 2) + (size_t(*ip++) << 2);
        if (!copyMatch(dist, 2)) {
          return false;
        }
        skipToMatchNext = true;
        continue;
      }
      // A 3-byte match just past 2 kB, right after a long literal run.
      if (!needIn(1)) {
        return false;
      }
      next = t & 3;
      dist = 1 + 0x0800 + (t >> 2) + (size_t(*ip++) << 2);
      t = 3;
    } else if (t >= 64) {
      // M2: 3-8 bytes within 2 kB.
      if (!needIn(1)) {
        return false;
      }
      next = t & 3;
      dist = 1 + ((t >> 2) & 7) + (size_t(*ip++) << 3);
      t = (t >> 5) - 1 + 2;
    } else if (t >= 32) {
      // M3: within 16 kB.
      t = (t & 31) + 2;
      if (t == 2 && !longLength(31, t)) {
        return false;
      }
      if (!needIn(2)) {
        return false;
      }
      const size_t d = size_t(ip[0]) | (size_t(ip[1]) << 8);
      ip += 2;
      dist = 1 + (d >> 2);
      next = d & 3;
    } else {
      // M4: 16 kB to 48 kB back, or the end of the stream.
      const size_t high = (t & 8) << 11;
      t = (t & 7) + 2;
      if (t == 2 && !longLength(7, t)) {
        return false;
      }
      if (!needIn(2)) {
        return false;
      }
      const size_t d = size_t(ip[0]) | (size_t(ip[1]) << 8);
      ip += 2;
      next = d & 3;
      const size_t back = high + (d >> 2);
      if (back == 0) {
        // The end marker: a 3-byte "match" at distance 0.
        return t == 3 && ip == ipEnd && op == opEnd;
      }
      dist = back + 0x4000;
    }
    if (!copyMatch(dist, t)) {
      return false;
    }
    skipToMatchNext = true;
  }
}

bool DecompressSegmented(const uint8_t* in, size_t inSize, std::vector< uint8_t >& out) {
  if (inSize < 4) {
    return false;
  }
  const size_t total =
      (size_t(in[0]) << 24) | (size_t(in[1]) << 16) | (size_t(in[2]) << 8) | size_t(in[3]);
  // No Echoes resource is near this; a larger one is a misread.
  if (total > (64u << 20)) {
    return false;
  }
  out.assign(total, 0);
  size_t pos = 4;
  size_t done = 0;
  while (done < total) {
    if (inSize - pos < 2) {
      return false;
    }
    const int16_t size = int16_t(uint16_t((in[pos] << 8) | in[pos + 1]));
    pos += 2;
    const size_t want = total - done < 0x4000 ? total - done : 0x4000;
    if (size < 0) {
      // Stored: the segment's bytes as they are.
      const size_t n = size_t(-int32_t(size));
      if (n > total - done || inSize - pos < n) {
        return false;
      }
      std::memcpy(out.data() + done, in + pos, n);
      done += n;
      pos += n;
      continue;
    }
    const size_t n = size_t(size);
    if (n == 0 || inSize - pos < n) {
      return false;
    }
    if (!Decompress1x(in + pos, n, out.data() + done, want)) {
      // A segment whose LZO came out larger than the data is stored instead; with a positive size
      // that is exactly the bytes it inflates to.
      if (n != want) {
        return false;
      }
      std::memcpy(out.data() + done, in + pos, n);
    }
    done += want;
    pos += n;
  }
  return true;
}

} // namespace PortLzo
