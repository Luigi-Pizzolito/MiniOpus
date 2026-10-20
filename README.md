# MiniOpus

A single-header, single-file **Ogg Opus decoder (and optional encoder) in C**, in
the shape of the [Dr\*libs](https://github.com/mackron/dr_libs) — `dr_flac`'s
handle with transparent fields, its read/seek/tell callbacks, its allocation
callbacks, its one-shot helpers — with **libopus** underneath.

It is a **port**, not a re-derivation. The Ogg layer is a C translation of
[micro-ogg-demuxer](https://github.com/esphome-libs/micro-ogg-demuxer)'s `OggDemuxer`
(Kevin Ahrendt, Apache-2.0). The decode, seek and comment-parsing layers are
ported from UMP-STMH7, the STM32H7 music player that ran that demuxer inside a
96 KB decoder slot: a fixed feed window in front of it, page-header-level
access, granule bisection to seek, and a comment parser that works off declared
lengths so a 2 MB embedded cover is never read into memory. Keeping those semantics intact is the point: the memory
envelope and the I/O cost are the ones that were proven on hardware.

```c
#define MINIOPUS_IMPLEMENTATION   /* in exactly one .c file */
#include "miniopus.h"

MiniOpus* pOpus = miniopus_open_file("track.ogg", NULL);
miniopus_int16 pcm[1024 * 2];
while (miniopus_read_pcm_frames_s16(pOpus, 1024, pcm) > 0) play(pcm);
miniopus_close(pOpus);
```

- **License:** Apache-2.0 (whole repository — see [LICENSE](LICENSE)). Provenance
  of every part is in [NOTICE](NOTICE).
- **Dependency:** libopus (`pkg-config opus`), tested against 1.6.1. Not bundled.
- **Built and tested:** C11, `-Wall -Wextra -Wshadow -Wconversion` clean, and
  clean in every configuration build (see [Build variants](#build-variants)).


## Contents

- [What it does](#what-it-does)
- [Quickstart](#quickstart)
- [API](#api)
- [Configuration defines](#configuration-defines)
- [Semantics: duration, pre-skip and seeking](#semantics-duration-pre-skip-and-seeking)
- [Memory](#memory)
- [I/O cost and speed](#io-cost-and-speed)
- [Conformance and tests](#conformance-and-tests)
- [What was ported, and what was corrected](#what-was-ported-and-what-was-corrected)
- [Limitations](#limitations)


## What it does

| | |
|---|---|
| Container | Ogg demux with the full RFC 3533 / micro-ogg state machine: zero-copy packets, a bounded assembly buffer that **skips** oversized packets instead of buffering them, a streaming raw-body mode, BOS / serial / page-sequence / continuation / EOS validation |
| Codec | libopus, s16 / s32 / f32 interleaved output, mono and stereo, 48 kHz |
| Seeking | granule bisection over page boundaries, `resetForSeek` into the demuxer, decoder reset, decode-and-discard to the target frame |
| Metadata | streaming Vorbis comments, `METADATA_BLOCK_PICTURE` (RFC 7845 / FLAC picture block, raw or base64), `miniopus_find_tag()` for a targeted lookup |
| Encoding | optional Ogg Opus muxer + libopus encoder, granules written so the output is seekable and decodes bit-consistently with `opusdec`/opusfile |
| Broadcast | forward-only streams with `onSeek`/`onTell` set to `NULL` still decode end to end |
| Failure | malformed input fails cleanly: bounded, no leaks, `miniopus_last_ogg_error()` reports micro-ogg's code verbatim |


## Quickstart

Host build (Linux/macOS, needs libopus):

```sh
cc -std=c11 -O2 -DMINIOPUS_IMPLEMENTATION -I. your_file.c miniopus.c \
   -o your_app $(pkg-config --cflags --libs opus) -lm
```

Run the shipped examples and the test suite:

```sh
sh tests/build_and_run.sh          # unit suite + compile matrix + examples
OGGTEST_DIR=~/Music/opus sh tests/build_and_run.sh   # adds a corpus cross-check vs opusfile
```

Decode, inspect and seek ([examples/miniopus_decode_info.c](examples/miniopus_decode_info.c)):

```
$ build/decode_example build/miniopus_example.ogg 1.0
picture: ... (only when the file has one)
vendor: MiniOpus example
  TITLE = Rising Tone
channels=2  sampleRate=48000  sourceSampleRate=48000
duration=1.994 s  (95688 frames, seekable=yes)
seeked to 1.000 s (currentPCMFrame=48000)
TITLE (via find_tag): Rising Tone
read 9600 frames of 16-bit PCM
peak sample: 6979 / 32767
```

Encode and read back ([examples/miniopus_encode_file.c](examples/miniopus_encode_file.c)):

```
$ build/encode_example out.ogg
wrote out.ogg: 96000 frames encoded, decoder reports 95688 (1.994 s), channels=2
TITLE read back: Rising Tone
after seek to 47844: 47844 frames decoded, expected 47844 -> exact
```

That last line is the encoder's own conformance check: the granules it writes
match the frames the decoder reads back, frame for frame.


## API

Everything returns `NULL` / `0` / `MINIOPUS_FALSE` on failure, dr_flac style;
`miniopus_last_error()` and `miniopus_last_ogg_error()` say which failure it was.

**Handle fields** (read them, don't write them): `channels`, `sampleRate`
(always 48000), `sourceSampleRate` (OpusHead `input_sample_rate`),
`totalPCMFrameCount` (0 when unknown), `currentPCMFrame`.

| Function | Notes |
|---|---|
| `miniopus_open_file` / `miniopus_open_file_with_metadata` | `FILE*` based; removed by `MINIOPUS_NO_FILEIO` |
| `miniopus_open` / `miniopus_open_with_metadata` | your `onRead`/`onSeek`/`onTell`; seek and tell may both be `NULL` for a forward-only stream |
| `miniopus_open_memory` / `miniopus_open_memory_with_metadata` | decodes a buffer you already hold |
| `miniopus_read_pcm_frames_s16` / `_s32` / `_f32` | interleaved, `channels` samples per frame; a short read means EOF or error |
| `miniopus_seek_to_pcm_frame` | frame index; clamps past the end; forward-only on non-seekable streams |
| `miniopus_find_tag` | re-walks the comment header; needs a seekable stream |
| `miniopus_last_error`, `miniopus_last_ogg_error` | the MiniOpus code, and micro-ogg's result code verbatim (`-1` invalid capture, `-4` sequence error, `-10` mid-packet mode switch, …) |
| `miniopus_get_channels` and friends, `miniopus_is_seekable`, `miniopus_has_picture` | NULL-tolerant getters |
| `miniopus_open_file_and_read_pcm_frames_s16/_s32/_f32`, `miniopus_open_memory_and_read_...`, `miniopus_free` | one-shot decode into an allocated buffer |
| `miniopus_encoder_create`, `miniopus_encoder_create_file`, `miniopus_encoder_create_memory`, `miniopus_encoder_write_s16/_s32/_f32`, `miniopus_encoder_finalize`, `miniopus_encoder_get_memory`, `miniopus_encoder_close`, `miniopus_encode_pcm16_to_opus_file` | encoding; removed by `MINIOPUS_NO_ENCODER` |

Metadata arrives once at open (through the `_with_metadata` variants): one
`MINIOPUS_METADATA_TAGS` event with the buffered comments and an
`oversizedCommentCount`, and one `MINIOPUS_METADATA_PICTURE` event. A picture too
large to buffer is still reported with its type, MIME, dimensions and exact
size, with `pData == NULL` — the bytes are streamed past, not read.

Custom I/O is three functions, and they are the same shape dr_flac uses:

```c
miniopus_size_t  myRead (void* pUserData, void* pBufferOut, miniopus_size_t bytesToRead);
miniopus_bool32  mySeek(void* pUserData, miniopus_int64 byteOffset, miniopus_seek_origin origin);
miniopus_uint64  myTell(void* pUserData);          /* (miniopus_uint64)-1 on failure */
```


## Configuration defines

Compile-tested in every combination by `tests/build_and_run.sh`.

| Define | Effect | Default |
|---|---|---|
| `MINIOPUS_IMPLEMENTATION` | build the implementation, exactly once | — |
| `MINIOPUS_NO_FILEIO` | drop the `FILE*` helpers | off |
| `MINIOPUS_NO_SEEKING` | compile the granule bisection out; seeking becomes forward-only decode-and-discard | off |
| `MINIOPUS_NO_METADATA` | compile the Vorbis-comment parser out (`find_tag`, `has_picture` not built; the comment header is still walked past) | off |
| `MINIOPUS_NO_ENCODER` | drop the encoder and muxer | off |
| `MINIOPUS_CHECK_CRC` | verify Ogg page CRCs while demuxing | off |
| `MINIOPUS_READ_CHUNK` | feed window size | 4096 |
| `MINIOPUS_MIN_PACKET_BUFFER` | initial packet assembly buffer | 1024 |
| `MINIOPUS_MAX_AUDIO_PACKET` | largest packet assembled; larger ones are skipped whole | 8192 |
| `MINIOPUS_BISECT_SCRATCH` | page-scan scratch, allocated only for the duration of a seek | 8192 |
| `MINIOPUS_MAX_TAG_VALUE` | largest comment value buffered in RAM | 16384 |
| `MINIOPUS_MAX_TAG_STORE` | cap on the tag store handed to the callback | 32768 |
| `MINIOPUS_MAX_PICTURE_BYTES` | cap on decoded image bytes handed to the callback | 65536 |
| `MINIOPUS_PAGE_BODY_MAX` | muxer page body capacity | 32768 |
| `MINIOPUS_TRACE` | print the header map built at open, page validation failures, seek landings | off |

CRC verification is **off by default** because UMP-STMH7 ran its demuxer
unchecked: a torn page should ride through a decode rather than end a track. The
muxer always writes correct CRCs regardless.


## Semantics: duration, pre-skip and seeking

These are the container semantics of the sources being ported, and they are the
ones the API is consistent with.

**A page's granule position is the end time of the last packet that completes on
it.** Pages that complete nothing (the middle of a 255-segment run) carry `-1`.

**Duration = the last page's granule − `pre_skip`.** Read at open by scanning
back from the end of the file for the page with the EOS flag; if a stream has no
EOS page, it comes instead from the newest granule the decode passes. Granules
are 48 kHz units, which is why PCM is decoded at 48 kHz.

**Pre-skip is trimmed off the decoded stream, not off the granules.** The
decoder counts `pre_skip` frames away from the head of what libopus produces. It
is *not* re-armed after a seek: the encoder's look-ahead padding only exists at
the start of the stream. (See [corrections](#what-was-ported-and-what-was-corrected).)

**A seek** is UMP-STMH7's chain, in this order:

1. Bisect the page chain for the page whose granule is the greatest one below
   `target = frameIndex + pre_skip` — interpolated midpoint, windowed backward
   and forward page scans, forward hops over `-1` granule pages, a short
   backward refine.
2. Step the landing page back until it **both opens and closes a packet**. The
   demuxer cannot restart mid-packet, and a packet a page hands onward is only
   ever seen as its tail: those frames never get decoded while their samples stay
   in the granules, which would make the discard overshoot.
3. Restart the feed there — `resetForSeek(page sequence, stream serial)` primes
   the demuxer's validation so a mid-file page is not read as a new bitstream —
   and reset the libopus decoder.
4. Walk whole packets until the landing page's granule is reached, then decode
   and discard exactly `target − that granule` frames. The position lands on the
   target frame; `currentPCMFrame` states it.

The fuzz that survives is the codec's and is unavoidable for any seek that
restarts a decoder: the first packets after a landing are decoded without the
overlap history the original encode had.

**One documented divergence from opusfile.** ffmpeg's Lavf muxer starts its
audio granules 24 samples into its timeline. MiniOpus (like UMP-STMH7) subtracts
only `pre_skip`, so on those files it reports a duration 24 frames (0.5 ms)
longer than `op_pcm_total()`, and lands 24 frames earlier than opusfile for the
same target. Measured across five real files:

| File | MiniOpus total | opusfile total | seek landing vs opusfile |
|---|---|---|---|
| DECO 27 — Love Words IV | 10512672 | 10512648 | +24 frames, waveform correlation 0.9999 |
| DECO 27 — Love Words V | 10512672 | 10512648 | +24, 0.9999 |
| DECO 27 — Monitoring (Best Friend Remix) | 8764512 | 8764488 | +24, 0.9999 |
| Yukopi — 寝起きヤシの木 | 5600352 | 5600328 | +24, 0.9999 |
| Zako — Akita Neru | 9962592 | 9962568 | +24, 0.9999 |

Each library is internally consistent — in the corpus check, seeking to `n`
leaves exactly `total − n` frames under each library's own total. The
`+24` is the muxer's base, not drift.


## Memory

Everything MiniOpus allocates goes through the caller's `miniopus_allocation_callbacks`.
Measured with a tracking allocator (the suite asserts every ledger closes at
`live == 0`):

| Block | Size | When |
|---|---|---|
| `MiniOpus` handle | 40 B | at open |
| Internal state — feed window 4096 B, demuxer 504 B (incl. its 282 B page staging), decode state, header map, parser state | 5776 B | at open |
| Decode scratch — one 60 ms packet, 2 channels, float (`OPUS_MAX_FRAME`, as UMP-STMH7 sizes it) | 23040 B | at open |
| Comment head scratch (`MINIOPUS_MAX_TAG_VALUE + 64`) | 16448 B | at open |
| **Total live at open** | **45304 B** | independent of file size |
| Packet assembly buffer | 1024 B, growing geometrically to 8192 B | first buffered packet |
| Tag store | grows with the tags it holds, capped at 32768 B | comment header |
| Seek page-scan scratch | 8192 B | only during a seek, freed after |
| Encoder (when used) | 32768 B page body + PCM staging + 1275 B packet buffer | at encoder create |

libopus's own decoder state is not routed through the callbacks; on this host it
shows up as +64 KB RSS at open (measured), which includes it. On a target without
`malloc`, UMP-STMH7 redirects libopus's allocations with
`-Wl,--wrap=malloc,--wrap=free` — the same trick works here:

```sh
arm-none-eabi-gcc ... -Wl,--wrap=malloc -Wl,--wrap=free
```

Code size, `-O2`, x86-64 (text segment only):

| Build | Text |
|---|---|
| default (everything) | 49.2 KB |
| `MINIOPUS_NO_SEEKING` | 42.7 KB |
| `MINIOPUS_NO_METADATA` | 38.6 KB |
| no fileio, no metadata, no seeking, no encoder | 21.5 KB |


## I/O cost and speed

Measured on the host (Intel x86-64, `-O2`, page cache warm, files on ext4).

**Decoding.** 19.9 s of stereo Opus decoded in 0.021 s → **938× realtime**
through the f32 path. Real corpus files (mono and stereo, 117–219 s long) drain
at **349–384× realtime** while being cross-checked against opusfile.

**Opening a file.** The cheap header path walks the page chain with header-only
reads and hops oversized comments without reading them (micro-ogg's oversized
skip still reads every byte it skips; page-level access is what fixes that):

| File | Size | Reads | Seeks | Bytes read at open |
|---|---|---|---|---|
| Zako — Akita Neru | 4143 KB (1.5 MB cover) | 119 | 120 | 25.8 KB |
| Yukopi — 寝起きヤシの木 | 2450 KB (0.4 MB cover) | 116 | 117 | 24.9 KB |
| DECO 27 — Love Words V | 5590 KB (2.1 MB cover) | 140 | 141 | 31.7 KB |

**Seeking.** On a 688 KB fixture, 40 randomly targeted bisections average **56.2
window reads and 35.8 physical seeks** each — the same shape as the
~62 reads / ~45 seeks UMP-STMH7 measured on hardware. On the corpus files a
mid-file seek costs 22–24 reads and 15–18 seeks, of which the bulk is the audio
the discard has to decode: a seek re-reads the landing page plus the audio
between it and the target (bounded by the page span — a second of audio at
4 KB pages, ~10 s on a muxer that runs 255-segment pages).


## Conformance and tests

```
$ sh tests/build_and_run.sh
DONE: 125 checks, 0 failures

$ OGGTEST_DIR=~/Music/opus sh tests/build_and_run.sh
DONE: 186 checks, 0 failures        # +61 checks against the real files in that directory
```

What the 125 checks cover, on four fixtures the library encodes itself (stereo +
tags + cover, mono VOIP CBR, 16 kHz 60 ms frames, and a 20 s file whose comment
header carries an oversized blob and a cover that exceeds the buffering cap):

- durations, pre-skip, channel counts, and the round-trip identity between
  MiniOpus's muxer and MiniOpus's demuxer;
- a seek battery that asserts **exact** frame counts after each landing
  (`frames to EOF == total − target`), waveform agreement with the continuous
  decode, backward seeks, seeking to 0, to `total − 1`, and past the end;
- bisection staying bounded (no walk-the-file behaviour);
- metadata: tags in order past oversized comments, picture headers parsed from
  the front of a base64 block whose bytes are never buffered, exact reported
  picture sizes, `miniopus_find_tag` hits and misses, playback surviving a
  `find_tag`;
- broadcast streams with no seek/tell, forward-only seek, backward seek rejected;
- **byte-flip fuzzing** (120 mutated streams): no crash, no leak, and either the
  stream plays or it fails with a stated code;
- **allocation ledgers** per test section: every open closes at `live == 0`.

Cross-checks against **opusfile 0.12** (skipped automatically when pkg-config
cannot find it): identical pre-skip, identical declared totals, identical frame
counts, and **bit-identical decoded samples** (0 differing floats) on all four
fixtures.

Cross-checks against **real files** (`tests/corpus_check.c`): five Lavf-muxed
tracks with 0.4–2.1 MB embedded covers, 255-segment pages and `-1` granule
continuation pages — 0 failures, including the granule-base relationship above.


## What was ported, and what was corrected

Ported, with the source's names kept so the files can be read side by side:

| In MiniOpus | From |
|---|---|
| `miniopus__dm_*` demuxer: states, spans, zero-copy return, grow_buffer + skipping mode, offer_body_data, validate_stream_consistency, reset_for_seek, enforce_mode | micro-ogg `OggDemuxer` (`ogg_demuxer.{h,cpp}`) |
| feed window (`chunk`, `chunkBase`, `inOff/inLen`), `pull` / `pull_raw` | UMP-STMH7 `ogg_shim.{h,cpp}` |
| `miniopus__page_hdr_parse`, `miniopus__headmap_map`, `miniopus__headmap_scan_tags`, the packet cursor (`pkt_open/next/take/advance/finish`) | UMP-STMH7 `ogg_pages.{h,c}` |
| `miniopus__vs_*` streaming comment parser with the `skip` model | UMP-STMH7 `opus_meta.{h,c}` |
| `miniopus__parse_head`, `miniopus__handle_packet` (`pre_skip_left`, `resid`/`pktFrames`), `miniopus__read_frames` with its pull guard, `miniopus__find_total`, `miniopus__bisect_*`, `miniopus_seek_to_pcm_frame` | UMP-STMH7 `opus.c` |
| Ogg muxer, encoder API, granule writing, serial seeding | MiniOpus (no equivalent existed to port: UMP-STMH7 has no Opus exporter) |

Three deliberate corrections to the ported arithmetic, each commented where it
appears, each established by a test that fails without it:

1. **The seek's discard is measured from the end of the landing page, not from
   its start.** `target − landingGranule` frames is correct only once the landing
   page's own packets have been walked off; counting from the page's start
   under-discards by the page's span, which lands a whole page (up to ~10 s on
   255-segment pages) early. UMP-STMH7's `opus_seek` hides this by declaring the
   position afterwards; MiniOpus keeps the declaration but fixes the count, so the
   seek battery can assert exactness.
2. **Pre-skip is not re-armed after a seek.** Re-setting `pre_skip_left =
   pre_skip` on every landing throws away `pre_skip` extra frames each time.
   It is a stream-head property.
3. **A landing page must close a packet as well as open one.** UMP-STMH7's rule
   (applied when finding where audio starts) is applied to seek landings too.

And four places where MiniOpus deliberately differs from UMP-STMH7's decoder,
because the API is a general-purpose one:

- PCM is always delivered at 48 kHz. UMP-STMH7 creates its libopus decoder at
  OpusHead's `input_sample_rate`, which makes granules and emitted frames
  disagree on files that are not 48 kHz. `sourceSampleRate` is exposed instead.
- Mono stays mono (`channels == 1`); UMP-STMH7 up-mixes to stereo for its DMA path.
- OpusHead's `output_gain` is applied through `OPUS_SET_GAIN`, matching opusfile
  and `opusdec`; UMP-STMH7 ignores it.
- A 120 ms packet is decoded, not dropped. UMP-STMH7 clamps its fixed scratch
  (its C6) and skips what does not fit; MiniOpus keeps the same 23 KB default and
  grows the scratch only for a file that actually carries longer frames.


## Limitations

- **1 and 2 channels only.** Channel mapping families 1 and 2 (5.1, ambisonic,
  multistream) are refused at open with `MINIOPUS_UNSUPPORTED`; use libopus's
  multistream or projection APIs directly for those.
- **No re-encoding, no resampling, no in-buffer seeking of non-Ogg data.** Ogg
  only; no WebM/Matroska Opus, no `.opus` in MP4.
- `totalPCMFrameCount` is 0 on streams with no seek/tell or no reachable EOS page,
  and it follows the convention described above on files whose muxer offsets its
  granules.
- `miniopus_find_tag()` needs a seekable stream.
- CRC checking is opt-in; corruption inside audio data is tolerated by design
  (the decode rides through it) rather than treated as a fatal error.


## License

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE), which records that the
demuxer port derives from micro-ogg-demuxer (Copyright 2025 Kevin Ahrendt,
Apache-2.0) and that the decoder/seek/comment layers are ported from UMP-STMH7
(Copyright 2026 Luigi Pizzolito, licensed here under Apache-2.0 by its author).
libopus is an external dependency and is not included.
