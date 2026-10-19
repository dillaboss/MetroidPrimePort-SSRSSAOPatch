#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

// LZO1X decompression, for Metroid Prime 2: Echoes' PAKs (Prime 1 uses zlib). Echoes compresses a
// resource as LZO1X-999 in segments that each inflate to 0x4000 bytes (the last one to what is
// left), each led by a big-endian 16-bit size; a negative size is a segment stored as it is.
namespace PortLzo {

// Inflates one LZO1X stream into exactly `outSize` bytes at `out`. False on a stream that is
// malformed, reads past `in + inSize`, or does not fill `outSize` bytes exactly.
bool Decompress1x(const uint8_t* in, size_t inSize, uint8_t* out, size_t outSize);

// An Echoes compressed resource as it sits in a PAK: a big-endian u32 of the inflated size, then
// the segments (the rest is the PAK's 0xFF padding). False when the segments don't add up to it.
bool DecompressSegmented(const uint8_t* in, size_t inSize, std::vector< uint8_t >& out);

} // namespace PortLzo
