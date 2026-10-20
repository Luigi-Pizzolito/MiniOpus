/*
MiniOpus - a single-file Ogg Opus decoder (and optional encoder) in C.

Copyright 2026 Luigi Pizzolito

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.

No warranty, as the license says. See NOTICE for the provenance of the code
below, including the Apache-2.0 micro-ogg-demuxer that this implementation ports.


What this is
============
One .h, one .c, a flat C API and no hidden global state, in the shape of the
Dr*libs (mackron's dr_wav / dr_flac / dr_mp3): dr_flac's handle with transparent
fields, its read/seek/tell callbacks, its allocation callbacks, its
open-and-read-one-shot helpers. The codec is Xiph's reference libopus, which you
link against; everything else - Ogg demuxing, page-level seeking, Vorbis
comments, Ogg muxing - is plain C in this file.

It is a port, not a re-invention. The Ogg layer is a C translation of
micro-ogg-demuxer's OggDemuxer (Kevin Ahrendt, Apache-2.0). The decode, seek and
comment-parsing layers are ported from the author's STM32H7 music player
(UMP-STMH7), which drove that demuxer on an embedded target: a fixed feed window
in front of it, page-header-level access for reading headers and seeking, a
granule-timed seek that reinitialises the demuxer and decoder on a page and then
decodes and discards up to a page of audio, and a comment parser that works off
declared lengths instead of reading the album art. Both sources are Apache-2.0,
so this repository can be Apache-2.0 too.

Read it next to those sources and the functions, states and result codes should
line up. Where a behaviour here looks over-careful, the comment says which
hardware problem it came from.


Usage
=====
    #define MINIOPUS_IMPLEMENTATION
    #include "miniopus.h"

    MiniOpus* pOpus = miniopus_open_file("track.opus", NULL);
    if (pOpus == NULL) {
        // Failed to open. miniopus_last_error() / miniopus_last_ogg_error() say why.
    }

    miniopus_int16 buffer[1024 * 2];
    while (miniopus_read_pcm_frames_s16(pOpus, 1024, buffer) > 0) {
        // play
    }

    miniopus_close(pOpus);

Opus is always decoded at 48 kHz, which is what `sampleRate` reports and what
granules are counted in. What the file claims (OpusHead input_sample_rate,
usually 48000, sometimes 8000..24000 for voice) is exposed as
`sourceSampleRate`, for a caller that wants to resample - which is how
UMP-STMH7 feeds its 48k -> 96k output resampler.


Configuration defines
=====================
Set these before the file that defines MINIOPUS_IMPLEMENTATION includes this
header. tests/build_and_run.sh compile-tests every combination below.

    MINIOPUS_IMPLEMENTATION      Required to build the implementation. Exactly once.
    MINIOPUS_NO_FILEIO           Remove the FILE*-based helpers (miniopus_open_file,
                                 miniopus_encoder_create_file, miniopus_encode_pcm16_to_opus_file).
    MINIOPUS_NO_SEEKING          Compile the granule bisection out. miniopus_seek_to_pcm_frame()
                                 then decode-and-discards forward only, on every stream.
    MINIOPUS_NO_METADATA         Compile the Vorbis-comment parser out. miniopus_find_tag() and
                                 miniopus_has_picture() are not built. The comment header is still
                                 walked past - it has to be, to find where the audio starts.
    MINIOPUS_NO_ENCODER          Remove the encoder API and the Ogg muxer.
    MINIOPUS_CHECK_CRC           Verify Ogg page CRCs while demuxing. Off by default, because
                                 UMP-STMH7 ran its demuxer unchecked: a torn page should ride
                                 through a decode, not end a track. The muxer always writes
                                 correct CRCs regardless of this setting.
    MINIOPUS_READ_CHUNK          Feed window in bytes. Default 4096 (ogg_shim's window size).
    MINIOPUS_MIN_PACKET_BUFFER   Initial packet assembly buffer. Default 1024 (micro-ogg's).
    MINIOPUS_MAX_AUDIO_PACKET    Largest packet the demuxer will assemble. Default 8192 (micro-ogg's
                                 max_buffer_size, as ogg_shim configured it). Larger packets are
                                 skipped whole, never buffered.
    MINIOPUS_BISECT_SCRATCH      Bytes a seek borrows for page scanning, allocated only for the
                                 duration of that seek. Default 8192 (UMP-STMH7's pool block).
    MINIOPUS_MAX_TAG_VALUE       Largest comment value the parser buffers in RAM. Larger comments are
                                 parsed from their declared length, counted, and - on the cheap open
                                 path - seeked past without being read. A METADATA_BLOCK_PICTURE
                                 bigger than this is still reported with its type/MIME/dimensions
                                 and exact size, with pData == NULL. Default 16384.
    MINIOPUS_MAX_TAG_STORE       Cap on the tag store handed to a metadata callback. Default 32768.
    MINIOPUS_MAX_PICTURE_BYTES   Cap on the decoded image bytes handed to a metadata callback.
                                 Default 65536.
    MINIOPUS_PAGE_BODY_MAX       Muxer page body capacity when encoding. Default 32768.
    MINIOPUS_TRACE               Print the decoder's own diagnostics: the header map it built at
                                 open, page validation failures, where a seek landed. Off, and costs
                                 nothing when unset.
    MINIOPUS_API                 Linkage/storage class for the public API. Default extern.
    MINIOPUS_NO_STDIO_H / MINIOPUS_NO_STDLIB_H / MINIOPUS_NO_STRING_H
                                 Suppress those standard includes from the implementation. You must
                                 supply memcpy/memmove/memset/realloc/free/FILE yourself. Combine
                                 with MINIOPUS_NO_FILEIO if you do not want FILE at all.
*/

#ifndef MINIOPUS_H
#define MINIOPUS_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32) && defined(MINIOPUS_SHARED_LIB)
    #ifdef MINIOPUS_IMPLEMENTATION
        #define MINIOPUS_API __declspec(dllexport)
    #else
        #define MINIOPUS_API __declspec(dllimport)
    #endif
#else
    #if !defined(MINIOPUS_API)
        #define MINIOPUS_API extern
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
Basic types, named the way dr_flac names its own so the two APIs read the same.
*/
typedef int32_t     miniopus_bool32;
#define MINIOPUS_FALSE   0
#define MINIOPUS_TRUE    1

typedef int8_t      miniopus_int8;
typedef uint8_t     miniopus_uint8;
typedef int16_t     miniopus_int16;
typedef uint16_t    miniopus_uint16;
typedef int32_t     miniopus_int32;
typedef uint32_t    miniopus_uint32;
typedef int64_t     miniopus_int64;
typedef uint64_t    miniopus_uint64;
typedef size_t      miniopus_size_t;

/*
Result codes. Like dr_flac, the decoder's open/read/seek APIs signal failure
with NULL/0/MINIOPUS_FALSE; miniopus_result is exposed where the extra detail is
worth having (the encoder, miniopus_encode_pcm16_to_opus_file()).
*/
typedef enum
{
    MINIOPUS_OK                  = 0,
    MINIOPUS_ERROR               = 1,    /* generic: container corruption, codec error */
    MINIOPUS_INVALID_ARGUMENT    = 2,
    MINIOPUS_INVALID_FORMAT      = 3,    /* not an Ogg Opus stream (missing/bad OpusHead) */
    MINIOPUS_OUT_OF_MEMORY       = 4,
    MINIOPUS_UNSUPPORTED         = 5,    /* e.g. >2 channels, channel mapping family != 0 */
    MINIOPUS_INVALID_STATE       = 6     /* misuse: bad config, non-seekable stream, ... */
} miniopus_result;

/*
I/O callbacks, the same shape as dr_flac's.

onRead:     fill up to bytesToRead bytes into pBufferOut; return the number of
            bytes filled. 0 means EOF or failure - the demuxer ends the stream
            either way. If your storage layer distinguishes "no more data" from
            "the card broke", wrap onRead so a failure can be reported above the
            library; UMP-STMH7 does exactly this to keep a dead SD card from
            looking like the end of a track.
onSeek:     move the read position; return MINIOPUS_FALSE on failure.
onTell:     current byte position, or (miniopus_uint64)(-1) on failure.

onSeek and onTell may both be NULL for forward-only broadcast-style streams:
totalPCMFrameCount is then 0 (unknown), seek() decode-and-discards forward
only, and the stream still decodes end to end.
*/
typedef enum
{
    MINIOPUS_SEEK_SET = 0,
    MINIOPUS_SEEK_CUR = 1,
    MINIOPUS_SEEK_END = 2
} miniopus_seek_origin;

typedef miniopus_size_t  (*miniopus_read_proc) (void* pUserData, void* pBufferOut, miniopus_size_t bytesToRead);
typedef miniopus_bool32  (*miniopus_seek_proc) (void* pUserData, miniopus_int64 byteOffset, miniopus_seek_origin origin);
typedef miniopus_uint64  (*miniopus_tell_proc) (void* pUserData);

/*
Allocation callbacks, same shape as dr_flac's. Only MiniOpus's own allocations
flow through them: the MiniOpus object, the read window, the packet assembly
buffer, the float decode scratch, and (encoder builds) the muxer buffer and
the PCM staging block. libopus's internal allocations are its own - to pin
them on a malloc-less target, redirect malloc/free at link time the way
UMP-STMH7 does with -Wl,--wrap=malloc, or build libopus with a custom
allocator shim.
*/
typedef void* (*miniopus_malloc_proc)  (miniopus_size_t sizeInBytes, void* pUserData);
typedef void* (*miniopus_realloc_proc) (void* pOriginalBlock, miniopus_size_t sizeInBytes, void* pUserData);
typedef void  (*miniopus_free_proc)    (void* pOriginalBlock, void* pUserData);

typedef struct
{
    miniopus_malloc_proc  onMalloc;
    miniopus_realloc_proc onRealloc;
    miniopus_free_proc    onFree;
    void*               pUserData;
} miniopus_allocation_callbacks;

/*
Metadata, reported once at open through miniopus_meta_proc when using the
_with_metadata() variants.

Tags:   every Vorbis comment whose value fits MINIOPUS_MAX_TAG_VALUE arrives in
        one MINIOPUS_METADATA_TAGS event (key = raw bytes before the first '=',
        value NUL-terminated UTF-8). Comments larger than the cap are not
        delivered; they are counted in oversizedCommentCount. A large
        METADATA_BLOCK_PICTURE (album art) is still reported as a
        MINIOPUS_METADATA_PICTURE event with pData == NULL, carrying its
        type/MIME/dimensions read off the front of the block.
Picture: when the picture's base64 payload decodes to at most
        MINIOPUS_MAX_TAG_VALUE bytes it is delivered with the decoded image in
        pData and the block header parsed.
All pointers are valid only for the duration of the callback - copy what you
need, and do not stash pointers across events.
*/
typedef enum
{
    MINIOPUS_METADATA_TAGS     = 0,
    MINIOPUS_METADATA_PICTURE  = 1
} miniopus_metadata_type;

typedef struct
{
    const char* key;        /* e.g. "TITLE", raw case as stored */
    const char* value;      /* NUL-terminated UTF-8 */
} miniopus_tag;

typedef struct
{
    const char*       vendorString;             /* may be "" */
    miniopus_uint32     tagCount;
    const miniopus_tag* pTags;                    /* tagCount entries */
    miniopus_uint32     oversizedCommentCount;    /* not in pTags */
} miniopus_tags;

typedef struct
{
    miniopus_int32        pictureType;    /* Vorbis picture type; 3 = front cover; 0 = unknown */
    const char*         mimeType;       /* e.g. "image/jpeg" */
    const char*         description;    /* may be "" */
    miniopus_uint32       width;
    miniopus_uint32       height;
    miniopus_uint32       depth;
    miniopus_uint32       colors;
    const miniopus_uint8* pData;          /* decoded image bytes, or NULL when oversized */
    miniopus_uint64       dataSize;       /* image size in bytes, known even when pData == NULL */
} miniopus_picture;

typedef struct
{
    miniopus_metadata_type type;
    union
    {
        miniopus_tags    tags;
        miniopus_picture picture;
    } data;
} miniopus_metadata;

typedef void (*miniopus_meta_proc)(void* pUserData, const miniopus_metadata* pMetadata);

/*
The decoder. Like drflac, the struct is transparent: read the fields, do not
write them.

channels             1 or 2. Files with 3-8 channels (surround/ambisonic,
                     channel mapping families 1 and 2) are refused at open
                     with MINIOPUS_UNSUPPORTED; decode those with libopus's
                     multistream API directly if you need them.
sampleRate           Always 48000 - the rate PCM is returned at.
sourceSampleRate     OpusHead input_sample_rate. 48000 usually; 8000..24000
                     for narrowband/wideband voice. Resample from this if your
                     output runs at another rate.
totalPCMFrameCount   Duration in frames at sampleRate, read off the container:
                     the granule position of the last page minus pre-skip. 0 = unknown
                     (non-seekable, or a stream whose last page has not been reached).
                     This is UMP-STMH7's convention and it is the one this library's
                     positions are consistent with. It differs from opusfile on files
                     whose muxer starts the granule timeline above the encoder delay:
                     ffmpeg's Lavf muxer starts audio 24 granules in, so its files read
                     24 frames (0.5 ms) longer here than op_pcm_total() reports, and a
                     seek lands 24 frames early against opusfile. Both libraries are
                     internally consistent; measured across a corpus in the README.
currentPCMFrame      Frames handed out since open or the last successful seek.
*/
struct MiniOpus_Internal; /* implementation-owned; see miniopus.c */

typedef struct MiniOpus
{
    /* Public, transparent fields - read them, do not write them. */
    miniopus_uint32 channels;
    miniopus_uint32 sampleRate;
    miniopus_uint32 sourceSampleRate;
    miniopus_uint64 totalPCMFrameCount;
    miniopus_uint64 currentPCMFrame;

    /* Everything else is implementation-private. */
    struct MiniOpus_Internal* pInternal;
} MiniOpus;

MINIOPUS_API MiniOpus* miniopus_open(
    miniopus_read_proc onRead, miniopus_seek_proc onSeek, miniopus_tell_proc onTell,
    void* pUserData, const miniopus_allocation_callbacks* pAllocationCallbacks);

MINIOPUS_API MiniOpus* miniopus_open_with_metadata(
    miniopus_read_proc onRead, miniopus_seek_proc onSeek, miniopus_tell_proc onTell,
    miniopus_meta_proc onMeta, void* pUserData, const miniopus_allocation_callbacks* pAllocationCallbacks);

#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API MiniOpus* miniopus_open_file(const char* pPath, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API MiniOpus* miniopus_open_file_with_metadata(const char* pPath, miniopus_meta_proc onMeta, void* pUserData, const miniopus_allocation_callbacks* pAllocationCallbacks);
#endif

MINIOPUS_API MiniOpus* miniopus_open_memory(const void* pData, miniopus_size_t dataSize, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API MiniOpus* miniopus_open_memory_with_metadata(const void* pData, miniopus_size_t dataSize, miniopus_meta_proc onMeta, void* pUserData, const miniopus_allocation_callbacks* pAllocationCallbacks);

/*
Read interleaved PCM. Returns frames read (samples per channel). Anything
less than framesToRead means end of stream or error; the next call returns 0.
Mono files produce 1 sample per frame, stereo 2.
*/
MINIOPUS_API miniopus_uint64 miniopus_read_pcm_frames_s16(MiniOpus* pOpus, miniopus_uint64 framesToRead, miniopus_int16* pFramesOut);
MINIOPUS_API miniopus_uint64 miniopus_read_pcm_frames_s32(MiniOpus* pOpus, miniopus_uint64 framesToRead, miniopus_int32* pFramesOut);
MINIOPUS_API miniopus_uint64 miniopus_read_pcm_frames_f32(MiniOpus* pOpus, miniopus_uint64 framesToRead, float* pFramesOut);

/*
Seek to a PCM frame. Returns MINIOPUS_FALSE on failure.

On a seekable stream (onSeek + onTell) this is UMP-STMH7's chain. A bisection
over Ogg page boundaries finds the page whose granule is the greatest one below
target = frameIndex + pre_skip - an interpolated midpoint, windowed backward and
forward page scans, forward hops over pages whose granule is -1 (a muxer that
lets a 255-segment page continue a packet completes nothing on it, so a run of
them carries no position at all), and a short backward refine. The landing page
is then stepped back until it both opens and closes a packet: micro-ogg cannot
restart mid-packet, and a packet that a page hands onward is only ever seen as a
tail, so its frames never get decoded while its samples stay in the granules.

There the feed is restarted on that page (miniopus__shim_seek_to -> the demuxer's
resetForSeek with the page's own sequence and serial, which is what makes the
strict BOS/serial/sequence/continuation validation usable mid-file), the libopus
decoder is reset, and frames are decoded and thrown away - first whole packets
until the landing page's granule is reached, then (target - that granule) frames,
which is where the pre-skip-aware arithmetic lands exactly on the target. The
fuzz that remains is the codec's, and it is unavoidable for any seek that
restarts a decoder: the first packets after a landing are decoded without the
overlap history the original encode had.

On a non-seekable stream - or in a MINIOPUS_NO_SEEKING build - a seek
decode-and-discards forward only; seeking backwards fails.
*/
MINIOPUS_API miniopus_bool32 miniopus_seek_to_pcm_frame(MiniOpus* pOpus, miniopus_uint64 frameIndex);

MINIOPUS_API void miniopus_close(MiniOpus* pOpus);

/*
Convenience: open, decode every frame, return a malloc'd interleaved buffer
allocated with pAllocationCallbacks; release with miniopus_free(). *pTotalPCMFrameCount
receives the frame count actually read (which can be less than the declared
duration on truncated files).
*/
#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API miniopus_int16* miniopus_open_file_and_read_pcm_frames_s16(const char* pPath, miniopus_uint32* pChannels, miniopus_uint32* pSampleRate, miniopus_uint64* pTotalPCMFrameCount, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API miniopus_int32* miniopus_open_file_and_read_pcm_frames_s32(const char* pPath, miniopus_uint32* pChannels, miniopus_uint32* pSampleRate, miniopus_uint64* pTotalPCMFrameCount, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API float*        miniopus_open_file_and_read_pcm_frames_f32(const char* pPath, miniopus_uint32* pChannels, miniopus_uint32* pSampleRate, miniopus_uint64* pTotalPCMFrameCount, const miniopus_allocation_callbacks* pAllocationCallbacks);
#endif

MINIOPUS_API miniopus_int16* miniopus_open_memory_and_read_pcm_frames_s16(const void* pData, miniopus_size_t dataSize, miniopus_uint32* pChannels, miniopus_uint32* pSampleRate, miniopus_uint64* pTotalPCMFrameCount, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API miniopus_int32* miniopus_open_memory_and_read_pcm_frames_s32(const void* pData, miniopus_size_t dataSize, miniopus_uint32* pChannels, miniopus_uint32* pSampleRate, miniopus_uint64* pTotalPCMFrameCount, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API float*        miniopus_open_memory_and_read_pcm_frames_f32(const void* pData, miniopus_size_t dataSize, miniopus_uint32* pChannels, miniopus_uint32* pSampleRate, miniopus_uint64* pTotalPCMFrameCount, const miniopus_allocation_callbacks* pAllocationCallbacks);

MINIOPUS_API void miniopus_free(void* pPtr, const miniopus_allocation_callbacks* pAllocationCallbacks);

/*
Query helpers (all NULL-tolerant; *value untouched when pOpus is NULL).
*/
MINIOPUS_API miniopus_bool32 miniopus_get_channels           (const MiniOpus* pOpus, miniopus_uint32* pChannels);
MINIOPUS_API miniopus_bool32 miniopus_get_sample_rate        (const MiniOpus* pOpus, miniopus_uint32* pSampleRate);
MINIOPUS_API miniopus_bool32 miniopus_get_source_sample_rate (const MiniOpus* pOpus, miniopus_uint32* pSourceSampleRate);
MINIOPUS_API miniopus_bool32 miniopus_get_total_pcm_frames   (const MiniOpus* pOpus, miniopus_uint64* pTotalPCMFrameCount);
MINIOPUS_API miniopus_bool32 miniopus_get_current_pcm_frame  (const MiniOpus* pOpus, miniopus_uint64* pCurrentPCMFrame);
MINIOPUS_API miniopus_bool32 miniopus_is_seekable            (const MiniOpus* pOpus);
#ifndef MINIOPUS_NO_METADATA
MINIOPUS_API miniopus_bool32 miniopus_has_picture            (const MiniOpus* pOpus);
#endif

/*
What went wrong, when something did. miniopus_last_ogg_error() reports the
micro-ogg result code verbatim (-1 invalid capture, -4 sequence error,
-10 mid-packet mode switch, ...) - the same number UMP-STMH7's logs print.
*/
MINIOPUS_API miniopus_result miniopus_last_error(const MiniOpus* pOpus);
MINIOPUS_API int            miniopus_last_ogg_error(const MiniOpus* pOpus);

/*
Most players only want a handful of Vorbis comments, and only a few of the
metadata events. miniopus_find_tag() re-walks the comment header through the page
chain - reading comment heads, and hopping oversized comments without reading
them, the same trick that keeps a 2 MB album-art blob off a 192 KB heap - and
copies a matching value into your buffer. Keys are matched case-insensitively,
the way Vorbis wants. Needs a seekable stream: a broadcast stream has already
streamed its comments past and cannot go back for them. The feed window's place
in the stream is restored afterwards, so playback continues where it left off.
*/
#ifndef MINIOPUS_NO_METADATA
MINIOPUS_API miniopus_bool32 miniopus_find_tag(MiniOpus* pOpus, const char* pKey, char* pValueOut, miniopus_size_t valueBufferSize);
#endif

/*
Encoder. Needs libopus's encoder side (the standard library has it). Define
MINIOPUS_NO_ENCODER to remove the whole section.

Encodes interleaved PCM at one of Opus's five native input rates
(8000/12000/16000/24000/48000 Hz - resample first if your source is e.g.
44100; UMP-STMH7's pipeline has an output resampler for the same reason in
reverse). The output is a conformant Ogg Opus stream: OpusHead with the
encoder's real look-ahead as pre-skip, an OpusTags comment header, and 48 kHz
granule accounting - so what the encoder writes is seekable by
miniopus_seek_to_pcm_frame(), and decodes bit-consistently with opusdec and
opusfile (verified in the test suite).

    MiniOpusEncoder* pEnc = miniopus_encoder_create_file("out.opus", 48000, 2, NULL, NULL);
    miniopus_encoder_write_s16(pEnc, pInterleaved, frameCount);
    miniopus_encoder_close(pEnc);      // flushes the final page with EOS

pConfig == NULL means: 20 ms frames, MINIOPUS_APPLICATION_AUDIO, and libopus's
own default bitrate/VBR/complexity.
*/
#ifndef MINIOPUS_NO_ENCODER

typedef enum
{
    MINIOPUS_APPLICATION_VOIP                = 2048, /* == OPUS_APPLICATION_VOIP             */
    MINIOPUS_APPLICATION_AUDIO               = 2049, /* == OPUS_APPLICATION_AUDIO            */
    MINIOPUS_APPLICATION_RESTRICTED_LOWDELAY = 2051  /* == OPUS_APPLICATION_RESTRICTED_LOWDELAY */
} miniopus_application;

typedef struct
{
    miniopus_int32       bitrateBps;      /* 0 = libopus default; otherwise 500..512000 */
    miniopus_int32       vbr;             /* -1 = libopus default; 0 = CBR; 1 = VBR */
    miniopus_int32       complexity;      /* 0..10; -1 = default */
    miniopus_int32       frameMsX10;      /* frame size x10: 25/50/100/200/400/600; 0 = 200 (20 ms) */
    miniopus_application application;     /* MINIOPUS_APPLICATION_AUDIO unless set */
    const miniopus_tag*  pTags;           /* Vorbis comments for the OpusTags packet; may be NULL */
    miniopus_uint32      tagCount;
    const char*        pVendorString;   /* NULL = "MiniOpus" */
} miniopus_encoder_config;

struct MiniOpusEncoder_Internal; /* implementation-owned; see miniopus.c */

typedef struct MiniOpusEncoder
{
    /* Public, transparent fields. */
    miniopus_uint32 sampleRate;   /* PCM input rate, one of 8/12/16/24/48000 */
    miniopus_uint32 channels;     /* 1 or 2 */
    miniopus_uint32 frameMsX10;   /* frame size x10: 25/50/100/200/400/600 */

    struct MiniOpusEncoder_Internal* pInternal;
} MiniOpusEncoder;

typedef miniopus_size_t (*miniopus_write_proc)(void* pUserData, const void* pData, miniopus_size_t bytesToWrite);

MINIOPUS_API MiniOpusEncoder* miniopus_encoder_create(miniopus_write_proc onWrite, void* pUserData, miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAllocationCallbacks);

#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API MiniOpusEncoder* miniopus_encoder_create_file(const char* pPath, miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAllocationCallbacks);
#endif

/* Open-ended in-memory output: pages accumulate in a growable block; read
them back with miniopus_encoder_get_memory() after finalize/close. */
MINIOPUS_API MiniOpusEncoder* miniopus_encoder_create_memory(miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAllocationCallbacks);
MINIOPUS_API const miniopus_uint8* miniopus_encoder_get_memory(const MiniOpusEncoder* pEncoder, miniopus_size_t* pSizeOut);

MINIOPUS_API miniopus_bool32 miniopus_encoder_write_s16(MiniOpusEncoder* pEncoder, const miniopus_int16* pInterleaved, miniopus_uint64 frames);
MINIOPUS_API miniopus_bool32 miniopus_encoder_write_s32(MiniOpusEncoder* pEncoder, const miniopus_int32* pInterleaved, miniopus_uint64 frames);
MINIOPUS_API miniopus_bool32 miniopus_encoder_write_f32(MiniOpusEncoder* pEncoder, const float* pInterleaved, miniopus_uint64 frames);

/* Pad the tail to the last frame, write the final EOS page. Called by
close() automatically; call it yourself when you want miniopus_encoder_get_memory()
to see the complete stream before close(). Idempotent. */
MINIOPUS_API miniopus_result miniopus_encoder_finalize(MiniOpusEncoder* pEncoder);

#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API miniopus_result miniopus_encode_pcm16_to_opus_file(const char* pOutputPath, const miniopus_int16* pInterleaved, miniopus_uint64 frames, miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAllocationCallbacks);
#endif

MINIOPUS_API void miniopus_encoder_close(MiniOpusEncoder* pEncoder);

#endif /* MINIOPUS_NO_ENCODER */

#ifdef __cplusplus
}
#endif

#endif /* MINIOPUS_H */
