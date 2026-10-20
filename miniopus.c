/*
MiniOpus - implementation. Define MINIOPUS_IMPLEMENTATION in exactly one .c file and
link against libopus (Xiph's reference codec, BSD-3-Clause).

Copyright 2026 Luigi Pizzolito. Licensed under the Apache License, Version 2.0;
see LICENSE, and NOTICE for the provenance of the parts ported from
micro-ogg-demuxer (Kevin Ahrendt, Apache-2.0) and from UMP-STMH7's Opus decoder
(the author's own, granted here under Apache-2.0).

Architecture, and where each piece comes from
---------------------------------------------

The Ogg layer is a C port of micro-ogg-demuxer's OggDemuxer, kept recognisable:
the same three states (EXPECT_HEADER / ACCUM_HEADER / PROCESS_SEGMENTS), the same
282-byte inline page header, the same span model (begin_packet_span /
close_packet_span / scan_for_next_packet), the same zero-copy fast path when a
whole packet is already in the input buffer, the same bounded assembly buffer
that gives up and *skips* an oversized packet instead of growing past
max_buffer_size, the same streaming raw-body mode for reading a packet the buffer
cannot hold, the same BOS / serial / page-sequence / continued-flag / EOS
validation, and the same result codes (identical values; the names carry the MiniOpus
prefix) - including MOGG_RES_INVALID_MODE_SWITCH, which is
what a mid-packet jump from streaming to packet mode earns you (C24).

In front of it sits ogg_shim's feed window: one MINIOPUS_READ_CHUNK buffer, topped
up when the demuxer runs out of input, with the absolute offset of chunk[0]
tracked so a body slice can report where it came from.

Page-header-level access (the port of ogg_pages.c) is what makes reading a
seekable file cheap: the header packets are located by walking page sizes, the
OpusTags packet is parsed from its declared lengths with oversized comments
*advanced past* rather than read, and only the page the audio starts on is handed
to the demuxer (C27). On a real file with a 1.5 MB base64 cover inside a 2 MB
comment header, that is about 120 small reads and 30 KB of file bytes to open -
the cover is never touched.

Duration and seeking follow UMP-STMH7 exactly. A page's granule is the end time
of the last packet that completes on it, so duration is the last page's granule
minus pre-skip, read at open by scanning back from the end of the file for the
EOS page (and, if a stream has none, from the newest granule the decode passes).
A seek bisects the page chain for the greatest granule below target = frame +
pre-skip, restarts the demuxer there with resetForSeek(page sequence, serial),
resets the libopus decoder, walks off the landing page packet by packet, and
then decodes and discards (target - landing granule) frames. The pre-skip trim is
a counter over decoded frames (pre_skip_left), not a granule trick - and it is
applied in float units, which is the C1 bug this port exists to not repeat.

Memory: everything MiniOpus allocates goes through the caller's allocation
callbacks, and the fixed-size parts (page staging, feed window, decode scratch for
two channels of one 60 ms packet) live inside the handle, so a target can carve
the whole decoder out of one pool block the way UMP-STMH7 carves its 96 KB slot.
libopus's own allocations are its own; UMP-STMH7 redirects them with
-Wl,--wrap=malloc and the README covers the same trick here.
*/

#ifdef MINIOPUS_IMPLEMENTATION

#if defined(__unix__) || defined(__APPLE__)
    #if !defined(_XOPEN_SOURCE) && !defined(_POSIX_C_SOURCE) && !defined(_GNU_SOURCE)
        #define _XOPEN_SOURCE 700   /* for fseeko/ftello */
    #endif
#endif

#include "miniopus.h"

#ifndef MINIOPUS_NO_STDLIB_H
    #include <stdlib.h>
#endif
#ifndef MINIOPUS_NO_STRING_H
    #include <string.h>
#endif
#ifndef MINIOPUS_NO_MATH_H
    #include <math.h>
#endif
#include <opus.h>

#if !defined(MINIOPUS_NO_FILEIO) && !defined(MINIOPUS_NO_STDIO_H)
    #include <stdio.h>
#endif

#if defined(_MSC_VER)
    #define MO_lrintf   (long)lrintf
#else
    #define MO_lrintf   lrintf
#endif

/* ------------------------------------------------------------------ config */

#ifndef MINIOPUS_READ_CHUNK
    #define MINIOPUS_READ_CHUNK 4096
#endif
#ifndef MINIOPUS_MAX_AUDIO_PACKET
    #define MINIOPUS_MAX_AUDIO_PACKET 8192
#endif
#ifndef MINIOPUS_MAX_TAG_VALUE
    #define MINIOPUS_MAX_TAG_VALUE 16384
#endif
#ifndef MINIOPUS_MAX_TAG_STORE
    #define MINIOPUS_MAX_TAG_STORE 32768
#endif
/* Decoded pictures delivered to the metadata callback are capped separately
so an open() never mallocs a cover bigger than this. */
#ifndef MINIOPUS_MAX_PICTURE_BYTES
    #define MINIOPUS_MAX_PICTURE_BYTES 65536
#endif
/* Ogg muxer page body capacity when encoding. */
#ifndef MINIOPUS_PAGE_BODY_MAX
    #define MINIOPUS_PAGE_BODY_MAX 32768
#endif

#define MO_PAGE_HDR_MAX      (27 + 255)

/* micro-ogg's assembly buffer: starts small, never past MINIOPUS_MAX_AUDIO_PACKET. */
#ifndef MINIOPUS_MIN_PACKET_BUFFER
    #define MINIOPUS_MIN_PACKET_BUFFER 1024
#endif
/* The page-scan scratch a seek borrows (UMP pool-allocates 8 KB for the same job). */
#ifndef MINIOPUS_BISECT_SCRATCH
    #define MINIOPUS_BISECT_SCRATCH 8192
#endif
#define MO_MAX_DECODE_FRAMES 5760              /* 120 ms at 48 kHz */
#define MO_PAGE_HDR_MAX      (27 + 255)

/* UMP-STMH7's decode scratch: one 60 ms packet, two channels, float. */
#define MO_MAX_FRAME         2880
/* libopus permits 120 ms packets; scratch may grow to this, never beyond. */
#define MO_MAX_DECODE_FRAMES 5760
/* Packets pulled per read_frames call before the stream is called stalled. */
#define MO_READ_GUARD        4096
/* Header slices the streamed open fallback may walk (C24's budget). */
#define MINIOPUS_HEADER_SLICE_GUARD 4096

/* Bisection budgets, as UMP-STMH7 measured them on its bench library. */
#define MO_BISECT_ITERS      64                /* binary-search iterations */
#define MO_BISECT_SCAN_MAX   (128 * 1024)      /* windowed scan reach each way */
#define MO_BISECT_GRAN_HOPS  64                /* granule -1 hops */
#define MO_BISECT_REFINE     256               /* backward refine hops */

#define MO_GRANULE_NONE      ((miniopus_int64)-1)
#define MO_PIC_KEY         "METADATA_BLOCK_PICTURE"
#define MO_PIC_KEY_LEN     22   /* the comment key; '=' sits at index 22, the value starts at 23 */

/* ------------------------------------------------------------------ alloc */

static void* miniopus__malloc(const miniopus_allocation_callbacks* pAlloc, miniopus_size_t size)
{
    if (pAlloc && pAlloc->onMalloc) return pAlloc->onMalloc(size, pAlloc->pUserData);
    return malloc(size);
}

static void* miniopus__realloc(const miniopus_allocation_callbacks* pAlloc, void* p, miniopus_size_t size)
{
    if (pAlloc && pAlloc->onRealloc) return pAlloc->onRealloc(p, size, pAlloc->pUserData);
    return realloc(p, size);
}

static void miniopus__free(const miniopus_allocation_callbacks* pAlloc, void* p)
{
    if (pAlloc && pAlloc->onFree) { pAlloc->onFree(p, pAlloc->pUserData); return; }
    free(p);
}

/* ------------------------------------------------------------------- crc32
CRC-32/MPEG-2 (poly 0x04C11DB7, init 0, MSB-first, no final xor) - the Ogg
page checksum. Always used by the muxer; the demuxer verifies only when
MINIOPUS_CHECK_CRC is defined (UMP-STMH7 runs its demuxer unchecked: a torn
page is tolerated and the decode rides through it). */

static const miniopus_uint32* miniopus__crc_table(void)
{
    static miniopus_uint32 table[256];
    static int built = 0;
    if (!built) {
        for (int i = 0; i < 256; i++) {
            miniopus_uint32 c = (miniopus_uint32)i << 24;
            for (int k = 0; k < 8; k++) {
                c = (c & 0x80000000u) ? ((c << 1) ^ 0x04C11DB7u) : (c << 1);
            }
            table[i] = c;
        }
        built = 1;
    }
    return table;
}

static miniopus_uint32 miniopus__crc_update(miniopus_uint32 crc, const miniopus_uint8* p, miniopus_size_t n)
{
    const miniopus_uint32* table = miniopus__crc_table();
    for (miniopus_size_t i = 0; i < n; i++) {
        crc = (crc << 8) ^ table[((crc >> 24) ^ (miniopus_uint32)p[i]) & 0xFFu];
    }
    return crc;
}

#ifndef MINIOPUS_NO_METADATA
/* ---------------------------------------------------------------- base64 */

static int miniopus__b64val(miniopus_uint8 c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;   /* '=' and whitespace are skipped */
}

/* Decode pB64[0..n) into pOut (skipping '=' and whitespace). Returns bytes written. */
static miniopus_size_t miniopus__b64decode(const char* pB64, miniopus_size_t n, miniopus_uint8* pOut, miniopus_size_t outCap)
{
    miniopus_uint32 acc = 0; int accN = 0; miniopus_size_t outLen = 0;
    for (miniopus_size_t i = 0; i < n; i++) {
        int v = miniopus__b64val((miniopus_uint8)pB64[i]);
        if (v < 0) continue;
        acc = (acc << 6) | (miniopus_uint32)v;
        if (++accN == 4) {
            if (outLen + 3 > outCap) return outLen;
            pOut[outLen++] = (miniopus_uint8)(acc >> 16);
            pOut[outLen++] = (miniopus_uint8)(acc >> 8);
            pOut[outLen++] = (miniopus_uint8)acc;
            accN = 0;
        }
    }
    if (accN >= 2) {
        acc <<= (6 * (4 - accN));
        if (outLen < outCap) pOut[outLen++] = (miniopus_uint8)(acc >> 16);
        if (accN >= 3 && outLen < outCap) pOut[outLen++] = (miniopus_uint8)(acc >> 8);
    }
    return outLen;
}

#endif /* !MINIOPUS_NO_METADATA */

/* ------------------------------------------------------------- LE/BE helpers */

static miniopus_uint16 miniopus__le16(const miniopus_uint8* p) { return (miniopus_uint16)(p[0] | ((miniopus_uint16)p[1] << 8)); }
static miniopus_int16  miniopus__le16s(const miniopus_uint8* p) { return (miniopus_int16)miniopus__le16(p); }
static miniopus_uint32 miniopus__le32(const miniopus_uint8* p) { return (miniopus_uint32)p[0] | ((miniopus_uint32)p[1] << 8) | ((miniopus_uint32)p[2] << 16) | ((miniopus_uint32)p[3] << 24); }
static miniopus_int64  miniopus__le64(const miniopus_uint8* p)
{
    return (miniopus_int64)(miniopus_uint64)miniopus__le32(p) |
           ((miniopus_int64)(miniopus_int32)miniopus__le32(p + 4) << 32);
}
static miniopus_uint32 miniopus__be32(const miniopus_uint8* p) { return ((miniopus_uint32)p[0] << 24) | ((miniopus_uint32)p[1] << 16) | ((miniopus_uint32)p[2] << 8) | (miniopus_uint32)p[3]; }
static void miniopus__put_le32(miniopus_uint8* p, miniopus_uint32 v) { p[0] = (miniopus_uint8)v; p[1] = (miniopus_uint8)(v >> 8); p[2] = (miniopus_uint8)(v >> 16); p[3] = (miniopus_uint8)(v >> 24); }
static void miniopus__put_le16(miniopus_uint8* p, miniopus_uint16 v) { p[0] = (miniopus_uint8)v; p[1] = (miniopus_uint8)(v >> 8); }
static void miniopus__put_le64(miniopus_uint8* p, miniopus_int64 v) { miniopus__put_le32(p, (miniopus_uint32)(v & 0xFFFFFFFFu)); miniopus__put_le32(p + 4, (miniopus_uint32)((v >> 32) & 0xFFFFFFFFu)); }

#ifndef MINIOPUS_NO_METADATA

/* --------------------------------------------------------- vorbis comments
Streaming Vorbis-comment parser fed the OpusTags packet as slices: a byte
state machine so a 2 MB album-art comment costs only bytes *counted*, never
buffered (the head of every comment is still kept, which is what makes the
picture header parseable even when the image streams past unbuffered). */

#define MOVS_STAGE_MAGIC   0   /* "OpusTags" */
#define MOVS_STAGE_VLEN    2   /* vendor length (directly after the magic - see muxer note) */
#define MOVS_STAGE_VENDOR  3   /* vendor body */
#define MOVS_STAGE_COUNT   4   /* comment count */
#define MOVS_STAGE_CLEN    5   /* next comment length */
#define MOVS_STAGE_CBODY   6   /* comment body (collect, or head+count) */
#define MOVS_STAGE_DONE    7
#define MOVS_STAGE_BAD     8

typedef struct {
    miniopus_uint32 stage;
    miniopus_uint8  num[8]; miniopus_uint32 numN;   /* fixed-field assembly */
    miniopus_uint32 left;                          /* bytes left in current body field */
    miniopus_uint32 commentsLeft;
    miniopus_uint32 skip;                            /* bytes of a counted-too-big comment nothing will look at */

    miniopus_uint8* pBuf; miniopus_size_t bufCap;   /* comment head scratch */
    miniopus_size_t bufLen;
    miniopus_bool32 tooBig;

    miniopus_uint8* pStore; miniopus_size_t storeLen, storeCap;   /* "key\0value\0" records */
    miniopus_uint32 tagsStored;
    miniopus_uint32 oversizedCount;
    miniopus_uint32 storeDropped;

    char vendor[64];

    miniopus_bool32 picFound;
    miniopus_bool32 picBuffered;                   /* whole comment fit the cap */
    miniopus_uint8* pPicVal; miniopus_size_t picValLen;   /* full base64 value copy (small pictures only) */
    miniopus_int32  picType; char picMime[256]; char picDesc[256];
    miniopus_uint32 picW, picH, picDepth, picColors, picDataLen;
    miniopus_bool32 picDataLenKnown;

    const char*   pFindKey;   /* find mode: match this key, stop at it */
    char*         pFindOut; miniopus_size_t findOutCap;
    miniopus_bool32 findHit;
} miniopus_vorbis;

static void miniopus__vs_reset(miniopus_vorbis* vs, const char* pFindKey, char* pFindOut, miniopus_size_t findOutCap)
{
    /* Parse state restarts; the comment-head scratch does not. Zeroing it out of
     * the struct here orphaned 16 KB on every open that walked the header map
     * twice (the open, then a find_tag). */
    miniopus_uint8* pBuf = vs->pBuf;
    miniopus_size_t bufCap = vs->bufCap;
    memset(vs, 0, sizeof(*vs));
    vs->pBuf = pBuf; vs->bufCap = bufCap;
    vs->stage = MOVS_STAGE_MAGIC;
    vs->pFindKey = pFindKey; vs->pFindOut = pFindOut; vs->findOutCap = findOutCap;
}

static miniopus_bool32 miniopus__vs_ensure_store(miniopus_vorbis* vs, const miniopus_allocation_callbacks* pAlloc, miniopus_size_t extra)
{
    if (vs->storeLen + extra <= vs->storeCap) return MINIOPUS_TRUE;
    if (vs->storeLen + extra > (miniopus_size_t)MINIOPUS_MAX_TAG_STORE) return MINIOPUS_FALSE;
    miniopus_size_t cap = vs->storeCap ? vs->storeCap : 512;
    while (cap < vs->storeLen + extra) cap *= 2;
    if (cap > (miniopus_size_t)MINIOPUS_MAX_TAG_STORE) cap = (miniopus_size_t)MINIOPUS_MAX_TAG_STORE;
    miniopus_uint8* p = (miniopus_uint8*)miniopus__realloc(pAlloc, vs->pStore, cap);
    if (!p) return MINIOPUS_FALSE;
    vs->pStore = p; vs->storeCap = cap;
    return MINIOPUS_TRUE;
}

static miniopus_bool32 miniopus__ci_equal(const char* a, const char* b)
{
    while (*a && *b) {
        char x = *a, y = *b;
        if (x >= 'a' && x <= 'z') x = (char)(x - 'a' + 'A');
        if (y >= 'a' && y <= 'z') y = (char)(y - 'a' + 'A');
        if (x != y) return MINIOPUS_FALSE;
        a++; b++;
    }
    return (*a == '\0' && *b == '\0') ? MINIOPUS_TRUE : MINIOPUS_FALSE;
}

static miniopus_bool32 miniopus__ci_prefix(const char* s, miniopus_size_t n, const char* prefix)
{
    miniopus_size_t pl = (miniopus_size_t)strlen(prefix);
    if (n < pl) return MINIOPUS_FALSE;
    for (miniopus_size_t i = 0; i < pl; i++) {
        char x = s[i];
        if (x >= 'a' && x <= 'z') x = (char)(x - 'a' + 'A');
        if (x != prefix[i]) return MINIOPUS_FALSE;
    }
    return MINIOPUS_TRUE;
}

/* Parse the binary METADATA_BLOCK_PICTURE header from decoded prefix bytes. */
static void miniopus__vs_parse_picture(miniopus_vorbis* vs, const miniopus_uint8* raw, miniopus_size_t rawLen)
{
    miniopus_size_t o = 0;
    if (rawLen < 8) return;
    vs->picType = (miniopus_int32)miniopus__be32(raw + o); o += 4;
    miniopus_uint32 m = miniopus__be32(raw + o); o += 4;
    if (m > 250) m = 250;                     /* sanity clamp: a hostile header cannot be trusted */
    if (o + m > rawLen) return;
    memcpy(vs->picMime, raw + o, m); vs->picMime[m] = '\0'; o += m;
    if (o + 4 > rawLen) return;
    miniopus_uint32 d = miniopus__be32(raw + o); o += 4;
    if (d > 250) d = 250;
    if (o + d > rawLen) return;
    memcpy(vs->picDesc, raw + o, d); vs->picDesc[d] = '\0'; o += d;
    if (o + 20 > rawLen) return;
    vs->picW = miniopus__be32(raw + o); o += 4;
    vs->picH = miniopus__be32(raw + o); o += 4;
    vs->picDepth = miniopus__be32(raw + o); o += 4;
    vs->picColors = miniopus__be32(raw + o); o += 4;
    vs->picDataLen = miniopus__be32(raw + o); o += 4;
    vs->picDataLenKnown = MINIOPUS_TRUE;
}

/* A picture comment is known by its key; pull the header out of its value head. */
static void miniopus__vs_picture_from_buf(miniopus_vorbis* vs, const miniopus_allocation_callbacks* pAlloc)
{
    vs->picFound = MINIOPUS_TRUE;
    miniopus_size_t valOff = MO_PIC_KEY_LEN + 1;   /* skip "KEY=" */
    miniopus_size_t avail = vs->bufLen > valOff ? vs->bufLen - valOff : 0;
    miniopus_uint8 raw[768];
    miniopus_size_t rn = miniopus__b64decode((const char*)vs->pBuf + valOff, avail, raw, sizeof(raw));
    miniopus__vs_parse_picture(vs, raw, rn);
    if (!vs->tooBig) {
        vs->picBuffered = MINIOPUS_TRUE;
        /* keep the full base64 value so the image itself can be delivered */
        miniopus_uint8* copy = (miniopus_uint8*)miniopus__malloc(pAlloc, vs->bufLen + 1);
        if (copy) { memcpy(copy, vs->pBuf + valOff, vs->bufLen - valOff); copy[vs->bufLen - valOff] = '\0';
                    vs->pPicVal = copy; vs->picValLen = vs->bufLen - valOff; }
    }
}

/* A comment finished, wholly in scratch: split key/value, collect or match. */
static void miniopus__vs_commit_small(miniopus_vorbis* vs, const miniopus_allocation_callbacks* pAlloc)
{
    miniopus_size_t n = vs->bufLen;
    miniopus_size_t eq = n;
    for (miniopus_size_t i = 0; i < n; i++) { if (vs->pBuf[i] == '=') { eq = i; break; } }

    if (eq == MO_PIC_KEY_LEN && miniopus__ci_prefix((const char*)vs->pBuf, eq, MO_PIC_KEY)) {
        miniopus__vs_picture_from_buf(vs, pAlloc);
        return;
    }

    char keyTmp[128];
    const char* pKey = keyTmp;
    const char* pVal = NULL;
    miniopus_size_t valN = 0;
    if (eq < 128) {
        memcpy(keyTmp, vs->pBuf, eq); keyTmp[eq] = '\0';
        if (eq < n) { vs->pBuf[n] = '\0'; pVal = (const char*)(vs->pBuf + eq + 1); valN = n - eq - 1; }
        else { pVal = ""; }
    } else {
        keyTmp[0] = '?'; keyTmp[1] = '\0';
        pVal = "";
    }

    if (vs->pFindKey) {
        if (miniopus__ci_equal(pKey, vs->pFindKey)) {
            if (vs->pFindOut && vs->findOutCap) {
                miniopus_size_t cpy = valN < vs->findOutCap - 1 ? valN : vs->findOutCap - 1;
                memcpy(vs->pFindOut, pVal, cpy); vs->pFindOut[cpy] = '\0';
            }
            vs->findHit = MINIOPUS_TRUE;
            vs->stage = MOVS_STAGE_DONE;   /* early stop: caller halts + cleans */
        }
        return;
    }

    miniopus_size_t kl = (miniopus_size_t)strlen(pKey) + 1;
    miniopus_size_t vl = valN + 1;
    if (miniopus__vs_ensure_store(vs, pAlloc, kl + vl)) {
        memcpy(vs->pStore + vs->storeLen, pKey, kl); vs->storeLen += kl;
        memcpy(vs->pStore + vs->storeLen, pVal, vl); vs->storeLen += vl;
        vs->tagsStored++;
    } else {
        vs->storeDropped++;
    }
}

/* A comment finished that was too big to buffer: count it; if its head said
picture, the head is enough to report the picture. */
static void miniopus__vs_finish_big(miniopus_vorbis* vs)
{
    vs->oversizedCount++;
    if (vs->bufLen > MO_PIC_KEY_LEN && vs->pBuf[MO_PIC_KEY_LEN] == '=' &&
        miniopus__ci_prefix((const char*)vs->pBuf, MO_PIC_KEY_LEN, MO_PIC_KEY)) {
        miniopus__vs_picture_from_buf(vs, NULL);   /* tooBig: pPicVal stays NULL */
    }
}

static miniopus_bool32 miniopus__vs_le4(miniopus_vorbis* vs, const miniopus_uint8** pp, miniopus_size_t* pn, miniopus_uint32* pOut)
{
    while (*pn > 0 && vs->numN < 4) { vs->num[vs->numN++] = *(*pp)++; (*pn)--; }
    if (vs->numN < 4) return MINIOPUS_FALSE;
    *pOut = miniopus__le32(vs->num); vs->numN = 0;
    return MINIOPUS_TRUE;
}

/* Feed one slice. Returns non-zero when the parse is finished (done or bad) -
caller decides whether that is a boundary or an early stop. */
static miniopus_bool32 miniopus__vs_feed(miniopus_vorbis* vs, const miniopus_uint8* p, miniopus_size_t n,
                                     const miniopus_allocation_callbacks* pAlloc)
{
    while (n > 0) {
        /* bulk-skip: album-art and vendor bytes counted, never looked at */
        if (vs->skip) {
            const miniopus_size_t k = n < vs->skip ? n : vs->skip;
            p += k; n -= k; vs->skip -= (miniopus_uint32)k;
            continue;
        }
        switch (vs->stage) {
        case MOVS_STAGE_MAGIC: {
            while (n > 0 && vs->numN < 8) { vs->num[vs->numN++] = *p++; n--; }
            if (vs->numN < 8) return MINIOPUS_FALSE;
            if (memcmp(vs->num, "OpusTags", 8) != 0) { vs->stage = MOVS_STAGE_BAD; return MINIOPUS_TRUE; }
            vs->numN = 0; vs->stage = MOVS_STAGE_VLEN;
            break;
        }
        case MOVS_STAGE_VLEN: {
            miniopus_uint32 v;
            if (!miniopus__vs_le4(vs, &p, &n, &v)) return MINIOPUS_FALSE;
            vs->left = v; vs->bufLen = 0;
            vs->stage = MOVS_STAGE_VENDOR;
            break;
        }
        case MOVS_STAGE_VENDOR: {
            miniopus_size_t take = vs->left < n ? vs->left : n;
            miniopus_size_t keep = take;
            if (vs->bufLen + keep > sizeof(vs->vendor) - 1) keep = sizeof(vs->vendor) - 1 - vs->bufLen;
            if (keep > 0) { memcpy(vs->vendor + vs->bufLen, p, keep); vs->bufLen += keep; }
            p += take; n -= take; vs->left -= (miniopus_uint32)take;
            if (vs->left == 0) { vs->vendor[vs->bufLen] = '\0'; vs->bufLen = 0; vs->stage = MOVS_STAGE_COUNT; }
            break;
        }
        case MOVS_STAGE_COUNT: {
            miniopus_uint32 v;
            if (!miniopus__vs_le4(vs, &p, &n, &v)) return MINIOPUS_FALSE;
            if (v == 0) { vs->stage = MOVS_STAGE_DONE; return MINIOPUS_TRUE; }
            if (v > 0x100000u) { vs->stage = MOVS_STAGE_BAD; return MINIOPUS_TRUE; }   /* absurd comment count: hostile */
            vs->commentsLeft = v;
            vs->stage = MOVS_STAGE_CLEN;
            break;
        }
        case MOVS_STAGE_CLEN: {
            miniopus_uint32 v;
            if (!miniopus__vs_le4(vs, &p, &n, &v)) return MINIOPUS_FALSE;
            if (v > 0x4000000u) { vs->stage = MOVS_STAGE_BAD; return MINIOPUS_TRUE; }  /* > 64 MB single comment: hostile */
            vs->left = v; vs->bufLen = 0;
            vs->tooBig = (v > (miniopus_uint32)MINIOPUS_MAX_TAG_VALUE) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
            vs->stage = MOVS_STAGE_CBODY;
            break;
        }
        case MOVS_STAGE_CBODY: {
            const miniopus_size_t cap = (miniopus_size_t)MINIOPUS_MAX_TAG_VALUE;
            miniopus_size_t take = vs->left < n ? vs->left : n;
            if (vs->tooBig && take > cap - vs->bufLen) take = (vs->bufLen < cap) ? (cap - vs->bufLen) : 0;
            if (take > 0) {
                memcpy(vs->pBuf + vs->bufLen, p, take);
                vs->bufLen += take; p += take; n -= take; vs->left -= (miniopus_uint32)take;
            }
            if (vs->left == 0) {
                if (vs->tooBig) miniopus__vs_finish_big(vs);
                else            miniopus__vs_commit_small(vs, pAlloc);
                vs->bufLen = 0;
                if (vs->stage == MOVS_STAGE_DONE) return MINIOPUS_TRUE;   /* find-hit early stop */
                if (--vs->commentsLeft == 0) { vs->stage = MOVS_STAGE_DONE; return MINIOPUS_TRUE; }
                vs->stage = MOVS_STAGE_CLEN;
                break;
            }
            /* UMP's META_CAP model: the comment's head is captured, so the comment
               is parsed and counted, and the rest of it is bytes nothing here will
               ever look at. Hand them to `skip` - the page-level open path seek
               past them without reading them (C27), the streaming path drops them. */
            if (vs->tooBig && vs->bufLen >= cap) {
                miniopus__vs_finish_big(vs);
                vs->skip = vs->left;
                vs->left = 0;
                vs->bufLen = 0;
                if (vs->stage == MOVS_STAGE_DONE) return MINIOPUS_TRUE;
                if (--vs->commentsLeft == 0) { vs->stage = MOVS_STAGE_DONE; return MINIOPUS_TRUE; }
                vs->stage = MOVS_STAGE_CLEN;
            }
            break;
        }
        case MOVS_STAGE_DONE:
        case MOVS_STAGE_BAD:
        default:
            return MINIOPUS_TRUE;
        }
    }
    return (vs->stage == MOVS_STAGE_DONE || vs->stage == MOVS_STAGE_BAD) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
}

/* ---- the accessors UMP's opus_meta_stream exposes (C27) ---- */

static miniopus_bool32 miniopus__vs_ensure_buf(miniopus_vorbis* vs, const miniopus_allocation_callbacks* pAlloc)
{
    if (vs->pBuf) return MINIOPUS_TRUE;
    vs->pBuf = (miniopus_uint8*)miniopus__malloc(pAlloc, (miniopus_size_t)MINIOPUS_MAX_TAG_VALUE + 64);
    if (!vs->pBuf) return MINIOPUS_FALSE;
    vs->bufCap = (miniopus_size_t)MINIOPUS_MAX_TAG_VALUE + 64;
    return MINIOPUS_TRUE;
}

static void miniopus__vs_free(miniopus_vorbis* vs, const miniopus_allocation_callbacks* pAlloc)
{
    if (!vs) return;
    if (vs->pPicVal) miniopus__free(pAlloc, vs->pPicVal);
    if (vs->pStore)  miniopus__free(pAlloc, vs->pStore);
    if (vs->pBuf)    miniopus__free(pAlloc, vs->pBuf);
    memset(vs, 0, sizeof(*vs));
}

/* Free the parsed store but keep the comment-head scratch: the open path still
 * needs somewhere to put the vendor string and picture header. */
static void miniopus__vs_free_keep(miniopus_vorbis* vs, const miniopus_allocation_callbacks* pAlloc)
{
    if (vs->pPicVal) { miniopus__free(pAlloc, vs->pPicVal); vs->pPicVal = NULL; vs->picValLen = 0; }
    if (vs->pStore)  { miniopus__free(pAlloc, vs->pStore);  vs->pStore = NULL; vs->storeLen = vs->storeCap = 0; }
}

static miniopus_uint32 miniopus__vs_pending_skip(const miniopus_vorbis* vs) { return vs->skip; }

static void miniopus__vs_skip(miniopus_vorbis* vs, miniopus_uint32 n)
{
    if (n > vs->skip) n = vs->skip;
    vs->skip -= n;
}

static miniopus_bool32 miniopus__vs_complete(const miniopus_vorbis* vs)
{
    return (vs->stage == MOVS_STAGE_DONE || vs->stage == MOVS_STAGE_BAD) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
}

#endif /* !MINIOPUS_NO_METADATA */

/* ============================================================== ogg demuxer
 *
 * A C port of micro-ogg-demuxer's OggDemuxer (Kevin Ahrendt, Apache-2.0) in
 * the form UMP-STMH7 vendors and drives it: RFC 3533 page demuxing with a
 * zero-copy fast path, a bounded assembly buffer that gives up and *skips*
 * oversized packets instead of buffering them, a streaming raw-body mode for
 * packets the buffer cannot hold, and the BOS / serial / page-sequence /
 * continued-flag validation that makes a mid-file restart safe to declare.
 *
 * Every function, state and result code below keeps micro-ogg's shape and meaning,
 * so this can be read side by side with ogg_demuxer.cpp: the states, spans and
 * function bodies match one to one, and the result codes are micro-ogg's numeric
 * values unchanged (its OGG_* names carry MiniOpus's MOGG_RES_* prefix here).
 */

/* micro-ogg's OggDemuxResult: same numeric values, MiniOpus-prefixed names. */
#define MOGG_RES_OK                          0
#define MOGG_RES_NEED_MORE_DATA              1
#define MOGG_RES_PACKET_SKIPPED              2
#define MOGG_RES_INVALID_CAPTURE            (-1)
#define MOGG_RES_INVALID_VERSION            (-2)
#define MOGG_RES_CRC_FAILED                 (-3)
#define MOGG_RES_STREAM_SEQUENCE_ERROR      (-4)
#define MOGG_RES_STREAM_BOS_ERROR           (-5)
#define MOGG_RES_STREAM_EOS_ERROR           (-6)
#define MOGG_RES_STREAM_SERIAL_MISMATCH     (-7)
#define MOGG_RES_STREAM_CONTINUATION_ERROR  (-8)
#define MOGG_RES_ALLOCATION_FAILED          (-9)
#define MOGG_RES_INVALID_MODE_SWITCH        (-10)
#define MOGG_RES_INVALID_INPUT              (-11)

#define MOGG_MAX_HEADER_SIZE   282                 /* 27 fixed bytes + a 255-entry lacing table */
#define MOGG_PAGE_HEADER_SIZE  27
#define MOGG_SEG_COUNT_OFFSET  26
#define MOGG_MAX_LACING        255

#define MOGG_ST_EXPECT_HEADER   0
#define MOGG_ST_ACCUM_HEADER    1
#define MOGG_ST_PROCESS_SEG     2

#define MOGG_MODE_UNSET   0
#define MOGG_MODE_PACKET  1
#define MOGG_MODE_DATA    2

typedef struct
{
    miniopus_size_t   size;                   /* total size of the packet in bytes */
    miniopus_bool32   complete;               /* the packet ends on this page */
    miniopus_uint8    segment_count;          /* segments the packet occupies */
} miniopus_packet_info;

typedef struct
{
    miniopus_uint8   header_type;             /* 0x01 continued, 0x02 BOS, 0x04 EOS */
    miniopus_int64   granule_position;
    miniopus_uint32  stream_serial;
    miniopus_uint32  page_sequence;
    miniopus_uint32  checksum;
    miniopus_uint8   segment_count;
} miniopus_page_header;

typedef struct
{
    const miniopus_uint8* data;               /* into the feed chunk, or the assembly buffer */
    miniopus_size_t       length;
    miniopus_int64        granule_position;
    miniopus_bool32       is_bos, is_eos, is_last_on_page, is_end_of_packet;
} miniopus_packet;

typedef struct
{
    int           oggResult;                /* micro-ogg's numeric result code, unchanged */
    miniopus_result result;                   /* the MiniOpus-facing code */
    miniopus_size_t bytesConsumed;
    miniopus_packet packet;
} miniopus_demux_state;

struct miniopus_demux
{
    /* Inline page-header buffer: 27 fixed bytes plus a full lacing table. */
    miniopus_uint8  staging[MOGG_MAX_HEADER_SIZE];
    miniopus_size_t stagingSize;
    miniopus_uint8* segTable;                 /* points at staging + 27 */

    miniopus_uint8* pBuf;                     /* packet assembly buffer, allocated lazily */
    miniopus_size_t bufCap, minBuf, maxBuf;

    miniopus_size_t bodyConsumed;             /* bytes of the current page body consumed */
    miniopus_size_t bodySize;                 /* page body size, from the lacing table */
    miniopus_size_t asmLen;                   /* bytes assembled of the packet in flight */
    miniopus_size_t spanLeft;                 /* unconsumed bytes of the span in flight */

    miniopus_int64  granule;                  /* the current page's granule position */
    miniopus_page_header page;

    miniopus_uint32 serial, expectSeq, crcAccum;

    miniopus_uint8  state, mode, segIdx;
    miniopus_packet_info span;
    miniopus_bool32 spanActive, assembling, skipping, prevContinued;
    miniopus_bool32 streamInit, pktIsBos, bosUsed, crcOn;

    miniopus_allocation_callbacks alloc;
};

static void miniopus__dm_init_state_common(struct miniopus_demux* d);

static void miniopus__dm_init(struct miniopus_demux* d, const miniopus_allocation_callbacks* pAlloc,
                            miniopus_size_t minBuf, miniopus_size_t maxBuf, miniopus_bool32 crcOn)
{
    memset(d, 0, sizeof(*d));
    d->segTable   = d->staging + MOGG_PAGE_HEADER_SIZE;
    d->minBuf     = minBuf ? minBuf : 1024;
    d->maxBuf     = (maxBuf < d->minBuf) ? d->minBuf : maxBuf;
    d->alloc      = *pAlloc;
    d->crcOn      = crcOn;
    d->state      = MOGG_ST_EXPECT_HEADER;
    d->mode       = MOGG_MODE_UNSET;
}

/* Same as micro-ogg's resetForSeek(): prime the sequence and serial so the
 * first page after a jump is not read as a new bitstream. */
static void miniopus__dm_reset_for_seek(struct miniopus_demux* d, miniopus_uint32 pageSequence, miniopus_uint32 streamSerial)
{
    miniopus__dm_init_state_common(d);
    d->serial      = streamSerial;
    d->expectSeq   = pageSequence;
    d->streamInit  = MINIOPUS_TRUE;
    d->bosUsed     = MINIOPUS_TRUE;
}

static void miniopus__dm_reset(struct miniopus_demux* d)
{
    miniopus__dm_init_state_common(d);
}

static void miniopus__dm_init_state_common(struct miniopus_demux* d)
{
    d->state = MOGG_ST_EXPECT_HEADER;
    d->stagingSize = 0;
    d->segIdx = 0;
    d->bodyConsumed = 0;
    d->bodySize = 0;
    d->asmLen = 0;
    d->assembling = MINIOPUS_FALSE;
    d->skipping = MINIOPUS_FALSE;
    d->span.size = 0; d->span.complete = MINIOPUS_FALSE; d->span.segment_count = 0;
    d->spanLeft = 0;
    d->spanActive = MINIOPUS_FALSE;
    d->prevContinued = MINIOPUS_FALSE;
    d->granule = 0;
    d->serial = 0;
    d->expectSeq = 0;
    d->streamInit = MINIOPUS_FALSE;
    d->pktIsBos = MINIOPUS_FALSE;
    d->bosUsed = MINIOPUS_FALSE;
    d->crcAccum = 0;
    d->mode = MOGG_MODE_UNSET;
}

/* ---- parse_page_header -------------------------------------------------- */

static int miniopus__dm_parse_page_header(const miniopus_uint8* data, miniopus_size_t dataLen,
                                        miniopus_page_header* header, miniopus_size_t* headerSize)
{
    if (dataLen < MOGG_PAGE_HEADER_SIZE) { *headerSize = 0; return MOGG_RES_NEED_MORE_DATA; }
    if (data[0] != 'O' || data[1] != 'g' || data[2] != 'g' || data[3] != 'S') { *headerSize = 0; return MOGG_RES_INVALID_CAPTURE; }
    if (data[4] != 0x00) { *headerSize = 0; return MOGG_RES_INVALID_VERSION; }

    header->header_type     = data[5];
    header->granule_position = miniopus__le64(data + 6);
    header->stream_serial   = miniopus__le32(data + 14);
    header->page_sequence   = miniopus__le32(data + 18);
    header->checksum        = miniopus__le32(data + 22);
    header->segment_count   = data[MOGG_SEG_COUNT_OFFSET];

    *headerSize = MOGG_PAGE_HEADER_SIZE + header->segment_count;
    if (dataLen < *headerSize) return MOGG_RES_NEED_MORE_DATA;
    return MOGG_RES_OK;
}

static miniopus_size_t miniopus__dm_calculate_body_size(const miniopus_uint8* table, miniopus_uint8 segmentCount)
{
    miniopus_size_t total = 0;
    for (miniopus_uint8 i = 0; i < segmentCount; i++) total += table[i];
    return total;
}

static miniopus_packet_info miniopus__dm_scan_for_next_packet(const struct miniopus_demux* d, miniopus_uint8 startSegmentIndex)
{
    miniopus_packet_info info;
    info.size = 0; info.complete = MINIOPUS_FALSE; info.segment_count = 0;

    for (miniopus_uint8 i = startSegmentIndex; i < d->page.segment_count; i++) {
        info.size += d->segTable[i];
        info.segment_count++;
        if (d->segTable[i] < MOGG_MAX_LACING) { info.complete = MINIOPUS_TRUE; break; }
    }
    return info;
}

static void miniopus__dm_begin_packet_span(struct miniopus_demux* d)
{
    d->span = miniopus__dm_scan_for_next_packet(d, d->segIdx);
    d->spanLeft = d->span.size;
}

/* The span's segment count includes a trailing zero-length lacing terminator
 * (packet size an exact multiple of 255), so the cursor lands on the next
 * packet's first segment - never mid-frame on a consumed terminator. */
static void miniopus__dm_close_packet_span(struct miniopus_demux* d)
{
    d->segIdx += d->span.segment_count;
    d->spanActive = MINIOPUS_FALSE;
}

static miniopus_bool32 miniopus__dm_current_page_ends_with_continued_packet(const struct miniopus_demux* d)
{
    return (d->page.segment_count > 0 && d->segTable[d->page.segment_count - 1] == MOGG_MAX_LACING)
        ? MINIOPUS_TRUE : MINIOPUS_FALSE;
}

/* ---- grow_buffer -------------------------------------------------------- */

/* 0 = GROW_OK, 1 = GROW_EXCEEDS_MAX, -1 = GROW_ALLOCATION_FAILED */
static int miniopus__dm_grow_buffer(struct miniopus_demux* d, miniopus_size_t neededSize)
{
    if (neededSize <= d->bufCap) return 0;
    if (neededSize > d->maxBuf) return 1;

    miniopus_size_t newCap = d->bufCap * 2;
    if (newCap < neededSize) newCap = neededSize;
    if (newCap > d->maxBuf) newCap = d->maxBuf;

    miniopus_uint8* p = (miniopus_uint8*)miniopus__realloc(&d->alloc, d->pBuf, newCap);
    if (!p) return -1;
    d->pBuf = p;
    d->bufCap = newCap;
    return 0;
}

static miniopus_result miniopus__dm_map_result(int oggResult)
{
    switch (oggResult) {
        case MOGG_RES_OK:
        case MOGG_RES_NEED_MORE_DATA:
        case MOGG_RES_PACKET_SKIPPED:               return MINIOPUS_OK;
        case MOGG_RES_INVALID_CAPTURE:
        case MOGG_RES_INVALID_VERSION:
        case MOGG_RES_CRC_FAILED:
        case MOGG_RES_STREAM_SEQUENCE_ERROR:
        case MOGG_RES_STREAM_BOS_ERROR:
        case MOGG_RES_STREAM_EOS_ERROR:
        case MOGG_RES_STREAM_SERIAL_MISMATCH:
        case MOGG_RES_STREAM_CONTINUATION_ERROR:   return MINIOPUS_INVALID_FORMAT;
        case MOGG_RES_ALLOCATION_FAILED:            return MINIOPUS_OUT_OF_MEMORY;
        case MOGG_RES_INVALID_MODE_SWITCH:
        case MOGG_RES_INVALID_INPUT:                return MINIOPUS_INVALID_STATE;
        default:                                  return MINIOPUS_ERROR;
    }
}

static void miniopus__dm_set_state(miniopus_demux_state* st, int oggResult)
{
    st->oggResult = oggResult;
    st->result = miniopus__dm_map_result(oggResult);
}

/* ---- CRC ---------------------------------------------------------------- */

static int miniopus__dm_validate_page_crc(const struct miniopus_demux* d)
{
    return (d->crcAccum == d->page.checksum) ? MOGG_RES_OK : MOGG_RES_CRC_FAILED;
}

static void miniopus__dm_seed_page_crc(struct miniopus_demux* d, const miniopus_uint8* headerData, miniopus_size_t headerSize)
{
    if (!d->crcOn) return;
    if (d->stagingSize == 0) memcpy(d->staging, headerData, headerSize);

    miniopus_uint8 saved[4];
    memcpy(saved, d->staging + 22, 4);
    memset(d->staging + 22, 0, 4);
    d->crcAccum = miniopus__crc_update(0, d->staging, headerSize);
    memcpy(d->staging + 22, saved, 4);
}

static miniopus_bool32 miniopus__dm_ensure_buffers_allocated(struct miniopus_demux* d, miniopus_demux_state* st)
{
    if (!d->pBuf) {
        d->bufCap = d->minBuf;
        miniopus_uint8* p = (miniopus_uint8*)miniopus__malloc(&d->alloc, d->bufCap);
        if (!p) { miniopus__dm_set_state(st, MOGG_RES_ALLOCATION_FAILED); return MINIOPUS_FALSE; }
        d->pBuf = p;
    }
    return MINIOPUS_TRUE;
}

/* ---- mode switching guard (C24) ----------------------------------------- */

static miniopus_bool32 miniopus__dm_between_packets(const struct miniopus_demux* d)
{
    if (d->assembling || d->skipping || d->spanActive) return MINIOPUS_FALSE;
    if (d->state != MOGG_ST_PROCESS_SEG) return d->prevContinued ? MINIOPUS_FALSE : MINIOPUS_TRUE;
    if (d->segIdx == 0) return d->prevContinued ? MINIOPUS_FALSE : MINIOPUS_TRUE;
    return MINIOPUS_TRUE;
}

static miniopus_bool32 miniopus__dm_enforce_mode(struct miniopus_demux* d, miniopus_uint8 requested, miniopus_demux_state* st)
{
    if (d->mode != MOGG_MODE_UNSET && d->mode != requested && !miniopus__dm_between_packets(d)) {
        miniopus__dm_set_state(st, MOGG_RES_INVALID_MODE_SWITCH);
        st->bytesConsumed = 0;
        memset(&st->packet, 0, sizeof(st->packet));
        st->packet.granule_position = MO_GRANULE_NONE;
        return MINIOPUS_FALSE;
    }
    d->mode = requested;
    return MINIOPUS_TRUE;
}

/* ---- finalize_page / validate_stream_consistency ------------------------ */

static int miniopus__dm_finalize_page(struct miniopus_demux* d)
{
#ifdef MINIOPUS_TRACE
    printf("[trace] finalize page seq=%u segs=%u lastSeg=%u -> prevCont=%s\n", d->page.page_sequence,
           d->page.segment_count, d->page.segment_count ? d->segTable[d->page.segment_count-1] : 0,
           (d->page.segment_count && d->segTable[d->page.segment_count-1] == 255) ? "yes" : "no");
#endif
    if (d->crcOn) {
        const int crc = miniopus__dm_validate_page_crc(d);
        if (crc != MOGG_RES_OK) return crc;
    }
    d->prevContinued = miniopus__dm_current_page_ends_with_continued_packet(d);
    d->state = MOGG_ST_EXPECT_HEADER;
    return MOGG_RES_OK;
}

static miniopus_bool32 miniopus__dm_validate_stream_consistency(struct miniopus_demux* d, miniopus_demux_state* st)
{
    if (!d->streamInit) {
        if (!(d->page.header_type & 0x02)) { miniopus__dm_set_state(st, MOGG_RES_STREAM_BOS_ERROR); return MINIOPUS_FALSE; }
        d->serial = d->page.stream_serial;
        d->expectSeq = d->page.page_sequence;
        d->streamInit = MINIOPUS_TRUE;
    } else {
        if (d->page.header_type & 0x02) { miniopus__dm_set_state(st, MOGG_RES_STREAM_BOS_ERROR); return MINIOPUS_FALSE; }
        if (d->page.stream_serial != d->serial) { miniopus__dm_set_state(st, MOGG_RES_STREAM_SERIAL_MISMATCH); return MINIOPUS_FALSE; }
        if (d->page.page_sequence != d->expectSeq) { miniopus__dm_set_state(st, MOGG_RES_STREAM_SEQUENCE_ERROR); return MINIOPUS_FALSE; }
    }
    d->expectSeq++;

    const miniopus_bool32 hasContinued = (d->page.header_type & 0x01) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    if (d->page.page_sequence > 0 && hasContinued != d->prevContinued) {
#ifdef MINIOPUS_TRACE
        printf("[trace] CONTINUATION MISMATCH seq=%u granule=%lld segs=%u lastSeg=%u flag=%02x prevCont=%d\n",
               d->page.page_sequence, (long long)d->page.granule_position, d->page.segment_count,
               d->page.segment_count ? d->segTable[d->page.segment_count - 1] : 0,
               d->page.header_type, (int)d->prevContinued);
#endif
        miniopus__dm_set_state(st, MOGG_RES_STREAM_CONTINUATION_ERROR);
        return MINIOPUS_FALSE;
    }

    if ((d->page.header_type & 0x04) && miniopus__dm_current_page_ends_with_continued_packet(d)) {
        miniopus__dm_set_state(st, MOGG_RES_STREAM_EOS_ERROR);
        return MINIOPUS_FALSE;
    }
    return MINIOPUS_TRUE;
}

/* ---- accumulate_header -------------------------------------------------- */

/* 0 = OK, 1 = NEED_MORE_DATA (st-> is filled). */
static int miniopus__dm_accumulate_header(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                        miniopus_size_t* bytesAdded, miniopus_demux_state* st)
{
    miniopus_size_t staged = d->stagingSize;
    *bytesAdded = 0;

    if (staged < MOGG_PAGE_HEADER_SIZE) {
        const miniopus_size_t need = MOGG_PAGE_HEADER_SIZE - staged;
        const miniopus_size_t toCopy = inputLen < need ? inputLen : need;
        if (toCopy > 0) {
            memcpy(d->staging + staged, input, toCopy);
            *bytesAdded = toCopy;
            staged += toCopy;
            d->stagingSize = staged;
        }
        if (staged < MOGG_PAGE_HEADER_SIZE) {
            st->bytesConsumed = *bytesAdded;
            miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA);
            return 1;
        }
    }

    const miniopus_uint8 segmentCount = d->staging[MOGG_SEG_COUNT_OFFSET];
    const miniopus_size_t fullHeaderSize = MOGG_PAGE_HEADER_SIZE + segmentCount;

    if (staged < fullHeaderSize) {
        const miniopus_size_t need = fullHeaderSize - staged;
        const miniopus_size_t avail = (inputLen > *bytesAdded) ? (inputLen - *bytesAdded) : 0;
        const miniopus_size_t toCopy = avail < need ? avail : need;
        if (toCopy > 0) {
            memcpy(d->staging + staged, input + *bytesAdded, toCopy);
            *bytesAdded += toCopy;
            staged += toCopy;
            d->stagingSize = staged;
        }
        if (staged < fullHeaderSize) {
            st->bytesConsumed = *bytesAdded;
            miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA);
            return 1;
        }
    }
    return 0;
}

/* ---- return paths ------------------------------------------------------- */

static void miniopus__dm_return_assembled_packet(struct miniopus_demux* d, miniopus_size_t bytesConsumed, miniopus_demux_state* st)
{
    st->packet.data = d->pBuf;
    st->packet.length = d->asmLen;

    st->packet.is_bos = d->pktIsBos;
    d->pktIsBos = MINIOPUS_FALSE;

    const miniopus_bool32 isLast = (d->segIdx >= d->page.segment_count) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    st->packet.is_last_on_page  = isLast;
    st->packet.is_end_of_packet = MINIOPUS_TRUE;
    st->packet.granule_position = isLast ? d->granule : MO_GRANULE_NONE;

    d->assembling = MINIOPUS_FALSE;
    d->asmLen = 0;
    st->bytesConsumed = bytesConsumed;

    if (isLast) {
        const int pageResult = miniopus__dm_finalize_page(d);
        if (pageResult != MOGG_RES_OK) { miniopus__dm_set_state(st, pageResult); return; }
        if (d->page.header_type & 0x04) st->packet.is_eos = MINIOPUS_TRUE;
    }
    miniopus__dm_set_state(st, MOGG_RES_OK);
}

static void miniopus__dm_handle_zero_copy_return(struct miniopus_demux* d, const miniopus_uint8* packetPtr,
                                               const miniopus_packet_info* packetInfo, miniopus_size_t additionalBytes,
                                               miniopus_demux_state* st)
{
    st->packet.data = packetPtr;
    st->packet.length = packetInfo->size;
    st->bytesConsumed = additionalBytes + packetInfo->size;

    d->segIdx += packetInfo->segment_count;
    d->bodyConsumed += packetInfo->size;
    if (d->crcOn) d->crcAccum = miniopus__crc_update(d->crcAccum, st->packet.data, st->packet.length);

    st->packet.is_bos = d->pktIsBos;
    d->pktIsBos = MINIOPUS_FALSE;

    const miniopus_bool32 isLast = (d->segIdx >= d->page.segment_count) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    st->packet.is_last_on_page  = isLast;
    st->packet.is_end_of_packet = MINIOPUS_TRUE;
    st->packet.granule_position = isLast ? d->granule : MO_GRANULE_NONE;

    if (isLast) {
        const int pageResult = miniopus__dm_finalize_page(d);
        if (pageResult != MOGG_RES_OK) { miniopus__dm_set_state(st, pageResult); return; }
        if (d->page.header_type & 0x04) st->packet.is_eos = MINIOPUS_TRUE;
    } else {
        d->state = MOGG_ST_PROCESS_SEG;
    }
    miniopus__dm_set_state(st, MOGG_RES_OK);
}

/* ---- handle_skipping_packet --------------------------------------------- */

static void miniopus__dm_handle_skipping_packet(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                              miniopus_demux_state* st)
{
    if (!d->spanActive) {
        miniopus__dm_begin_packet_span(d);
        d->spanActive = MINIOPUS_TRUE;
    }

    const miniopus_size_t toSkip = inputLen < d->spanLeft ? inputLen : d->spanLeft;
    if (toSkip > 0) {
        d->spanLeft -= toSkip;
        d->bodyConsumed += toSkip;
        st->bytesConsumed = toSkip;
        if (d->crcOn) d->crcAccum = miniopus__crc_update(d->crcAccum, input, toSkip);
    }

    if (d->spanLeft > 0) { miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA); return; }

    miniopus__dm_close_packet_span(d);

    if (!d->span.complete) {
        const int pageResult = miniopus__dm_finalize_page(d);
        miniopus__dm_set_state(st, (pageResult == MOGG_RES_OK) ? MOGG_RES_NEED_MORE_DATA : pageResult);
        return;
    }

    d->skipping = MINIOPUS_FALSE;

    const miniopus_bool32 isLast = (d->segIdx >= d->page.segment_count) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    if (isLast) {
        const int pageResult = miniopus__dm_finalize_page(d);
        if (pageResult != MOGG_RES_OK) { miniopus__dm_set_state(st, pageResult); return; }
    }
    /* A skipped packet still owns its place in the timeline: it completed on this
     * page, so the page's granule is the time its end sits at. Without this the
     * granule of a page ending a 2 MB comment header disappears and everything
     * downstream of it miscounts. */
    st->packet.is_last_on_page = isLast;
    st->packet.granule_position = isLast ? d->granule : MO_GRANULE_NONE;
    miniopus__dm_set_state(st, MOGG_RES_PACKET_SKIPPED);
}

/* ---- handle_assembling_packet ------------------------------------------- */

static void miniopus__dm_handle_assembling_packet(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                                miniopus_demux_state* st)
{
    if (!d->spanActive) {
        miniopus__dm_begin_packet_span(d);
        d->spanActive = MINIOPUS_TRUE;
    }

    /* Zero-size span: the packet's zero-length terminator leads this page. Flush
    the assembled packet before any page bookkeeping - finalizing first would
    merge this packet with the next one. */
    if (d->spanLeft == 0) {
        miniopus__dm_close_packet_span(d);
        miniopus__dm_return_assembled_packet(d, 0, st);
        return;
    }

    const miniopus_size_t toConsume = inputLen < d->spanLeft ? inputLen : d->spanLeft;
    if (toConsume == 0) { miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA); return; }

    const int grow = miniopus__dm_grow_buffer(d, d->asmLen + toConsume);
    if (grow != 0) {
        if (grow == 1) {
            d->skipping = MINIOPUS_TRUE;
            d->assembling = MINIOPUS_FALSE;
            d->asmLen = 0;
            miniopus__dm_handle_skipping_packet(d, input, inputLen, st);
            return;
        }
        miniopus__dm_set_state(st, MOGG_RES_ALLOCATION_FAILED);
        return;
    }

    memcpy(d->pBuf + d->asmLen, input, toConsume);
    d->asmLen += toConsume;
    d->bodyConsumed += toConsume;
    d->spanLeft -= toConsume;
    st->bytesConsumed = toConsume;
    if (d->crcOn) d->crcAccum = miniopus__crc_update(d->crcAccum, input, toConsume);

    if (d->spanLeft > 0) { miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA); return; }

    miniopus__dm_close_packet_span(d);

    if (d->span.complete) { miniopus__dm_return_assembled_packet(d, toConsume, st); return; }

    const int pageResult = miniopus__dm_finalize_page(d);
    miniopus__dm_set_state(st, (pageResult == MOGG_RES_OK) ? MOGG_RES_NEED_MORE_DATA : pageResult);
}

/* ---- handle_page_header ------------------------------------------------- */

/* 0 = OK (now processing segments), 1 = NEED_MORE_DATA, 2 = packet ready or error. */
static int miniopus__dm_handle_page_header(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                         miniopus_demux_state* st, miniopus_bool32 attemptPacketZeroCopy)
{
    const miniopus_uint8* headerData = NULL;
    miniopus_size_t headerDataLen = 0;
    const miniopus_size_t stagedBytes = d->stagingSize;
    miniopus_size_t bytesAddedToStaging = 0;

    if (stagedBytes > 0) {
        if (miniopus__dm_accumulate_header(d, input, inputLen, &bytesAddedToStaging, st) != 0) return 1;
        headerData = d->staging;
        headerDataLen = d->stagingSize;
    } else {
        headerData = input;
        headerDataLen = inputLen;
    }

    miniopus_size_t headerSize = 0;
    const int result = miniopus__dm_parse_page_header(headerData, headerDataLen, &d->page, &headerSize);

    if (result == MOGG_RES_NEED_MORE_DATA) {
        if (d->stagingSize == 0 && inputLen > 0) {
            const miniopus_size_t toStage = inputLen < MOGG_MAX_HEADER_SIZE ? inputLen : MOGG_MAX_HEADER_SIZE;
            memcpy(d->staging, input, toStage);
            d->stagingSize = toStage;
            st->bytesConsumed = toStage;
        }
        d->state = MOGG_ST_ACCUM_HEADER;
        miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA);
        return 1;
    }

    if (result != MOGG_RES_OK) { miniopus__dm_set_state(st, result); return 2; }

    if (headerData != d->staging) {
        memcpy(d->segTable, headerData + MOGG_PAGE_HEADER_SIZE, d->page.segment_count);
    }

    d->bodySize = miniopus__dm_calculate_body_size(d->segTable, d->page.segment_count);
    d->granule = d->page.granule_position;

    if (!miniopus__dm_validate_stream_consistency(d, st)) return 2;

    if (d->page.segment_count == 0) {
        miniopus__dm_seed_page_crc(d, headerData, headerSize);
        if (d->crcOn && miniopus__dm_validate_page_crc(d) != MOGG_RES_OK) {
            miniopus__dm_set_state(st, MOGG_RES_CRC_FAILED);
            return 2;
        }
        st->bytesConsumed = (bytesAddedToStaging > 0) ? bytesAddedToStaging : headerSize;
        d->stagingSize = 0;
        d->state = MOGG_ST_EXPECT_HEADER;
        miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA);
        return 1;
    }

    miniopus__dm_seed_page_crc(d, headerData, headerSize);

    d->segIdx = 0;
    d->bodyConsumed = 0;
    d->spanActive = MINIOPUS_FALSE;

    d->pktIsBos = ((d->page.header_type & 0x02) && !d->bosUsed) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    if (d->pktIsBos) d->bosUsed = MINIOPUS_TRUE;

    const miniopus_size_t bytesFromInputForHeader = (stagedBytes == 0) ? headerSize : bytesAddedToStaging;
    const miniopus_size_t remainingInInput = (bytesFromInputForHeader < inputLen) ? (inputLen - bytesFromInputForHeader) : 0;

    if (attemptPacketZeroCopy && remainingInInput > 0 && !d->assembling && !d->skipping) {
        const miniopus_packet_info firstPacket = miniopus__dm_scan_for_next_packet(d, 0);
        if (firstPacket.complete && remainingInInput >= firstPacket.size) {
            miniopus__dm_handle_zero_copy_return(d, input + bytesFromInputForHeader, &firstPacket, bytesFromInputForHeader, st);
            d->stagingSize = 0;
            return 2;
        }
    }

    st->bytesConsumed = (bytesAddedToStaging > 0) ? bytesAddedToStaging : headerSize;
    d->stagingSize = 0;
    d->state = MOGG_ST_PROCESS_SEG;
    miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA);
    return 0;
}

static void miniopus__dm_handle_zero_copy_path(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                             miniopus_demux_state* st)
{
    const miniopus_packet_info nextPacket = miniopus__dm_scan_for_next_packet(d, d->segIdx);

    if (nextPacket.complete && inputLen >= nextPacket.size) {
        miniopus__dm_handle_zero_copy_return(d, input, &nextPacket, 0, st);
        return;
    }
    if (inputLen == 0) { miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA); return; }

    d->span = nextPacket;
    d->spanLeft = nextPacket.size;
    d->spanActive = MINIOPUS_TRUE;
    d->assembling = MINIOPUS_TRUE;
    miniopus__dm_handle_assembling_packet(d, input, inputLen, st);
}

/* ---- get_next_packet / get_next_data ------------------------------------ */

/* Returns 1 = packet in st->packet, 0 = need more input (or a packet was
 * skipped), -1 = error (st->oggResult holds micro-ogg's code). */
static int miniopus__dm_get_next_packet(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                      miniopus_demux_state* st)
{
    memset(st, 0, sizeof(*st));
    st->packet.granule_position = MO_GRANULE_NONE;

    if (inputLen > 0 && !input) { miniopus__dm_set_state(st, MOGG_RES_INVALID_INPUT); return -1; }
    if (!miniopus__dm_enforce_mode(d, MOGG_MODE_PACKET, st)) return -1;
    if (!miniopus__dm_ensure_buffers_allocated(d, st)) return -1;

    if (d->state == MOGG_ST_EXPECT_HEADER || d->state == MOGG_ST_ACCUM_HEADER) {
        const int r = miniopus__dm_handle_page_header(d, input, inputLen, st, MINIOPUS_TRUE);
        if (r == 2) return (st->oggResult == MOGG_RES_OK) ? 1 : -1;
        return 0;
    }
    if (d->skipping) {
        miniopus__dm_handle_skipping_packet(d, input, inputLen, st);
        return (st->oggResult == MOGG_RES_OK) ? 1 : 0;
    }
    if (d->assembling) {
        miniopus__dm_handle_assembling_packet(d, input, inputLen, st);
        return (st->oggResult == MOGG_RES_OK) ? 1 : 0;
    }
    miniopus__dm_handle_zero_copy_path(d, input, inputLen, st);
    return (st->oggResult == MOGG_RES_OK) ? 1 : 0;
}

static void miniopus__dm_offer_body_data(struct miniopus_demux* d, const miniopus_uint8* body, miniopus_size_t bodyLen,
                                       miniopus_size_t headerBytes, miniopus_demux_state* st)
{
    /* No span in flight: rest the cursor on the next packet's span. A zero-size
    span leading a continuation page is the terminator of the packet continued
    from the previous page (its size is a multiple of 255) and is reported as a
    zero-length end-of-packet so the boundary is not lost; any other zero-size
    span is a genuine zero-length packet, stepped over silently. */
    if (!d->spanActive) {
        while (d->segIdx < d->page.segment_count) {
            miniopus__dm_begin_packet_span(d);
            if (d->spanLeft > 0) break;
            if (d->segIdx == 0 && d->prevContinued) {
                miniopus__dm_close_packet_span(d);

                st->packet.data = body;
                st->packet.length = 0;
                st->packet.granule_position = d->granule;
                st->packet.is_bos = d->pktIsBos;
                st->packet.is_eos = (d->page.header_type & 0x04) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
                st->packet.is_end_of_packet = MINIOPUS_TRUE;
                st->packet.is_last_on_page = (d->segIdx >= d->page.segment_count) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
                st->bytesConsumed = headerBytes;
                d->pktIsBos = MINIOPUS_FALSE;

                if (d->segIdx >= d->page.segment_count) {
                    const int pageResult = miniopus__dm_finalize_page(d);
                    if (pageResult != MOGG_RES_OK) { miniopus__dm_set_state(st, pageResult); return; }
                }
                miniopus__dm_set_state(st, MOGG_RES_OK);
                return;
            }
            miniopus__dm_close_packet_span(d);
        }

        if (d->segIdx >= d->page.segment_count) {
            const int pageResult = miniopus__dm_finalize_page(d);
            miniopus__dm_set_state(st, (pageResult == MOGG_RES_OK) ? MOGG_RES_NEED_MORE_DATA : pageResult);
            return;
        }
    }

    miniopus_size_t toOffer = bodyLen < d->spanLeft ? bodyLen : d->spanLeft;
    if (toOffer == 0) { miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA); return; }

    st->packet.data = body;
    st->packet.length = toOffer;
    st->packet.granule_position = d->granule;
    st->packet.is_bos = d->pktIsBos;
    st->packet.is_eos = (d->page.header_type & 0x04) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    st->packet.is_end_of_packet = (d->span.complete && toOffer == d->spanLeft) ? MINIOPUS_TRUE : MINIOPUS_FALSE;

    if (d->crcOn) d->crcAccum = miniopus__crc_update(d->crcAccum, body, toOffer);
    d->bodyConsumed += toOffer;
    d->spanLeft -= toOffer;
    if (d->spanLeft == 0) miniopus__dm_close_packet_span(d);
    else d->spanActive = MINIOPUS_TRUE;
    d->pktIsBos = MINIOPUS_FALSE;

    st->packet.is_last_on_page = (d->bodyConsumed >= d->bodySize) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    st->bytesConsumed = headerBytes + toOffer;

    if (d->bodyConsumed >= d->bodySize) {
        const int pageResult = miniopus__dm_finalize_page(d);
        if (pageResult != MOGG_RES_OK) { miniopus__dm_set_state(st, pageResult); return; }
    }
    miniopus__dm_set_state(st, MOGG_RES_OK);
}

static int miniopus__dm_get_next_data(struct miniopus_demux* d, const miniopus_uint8* input, miniopus_size_t inputLen,
                                    miniopus_demux_state* st)
{
    memset(st, 0, sizeof(*st));
    st->packet.granule_position = MO_GRANULE_NONE;

    if (inputLen > 0 && !input) { miniopus__dm_set_state(st, MOGG_RES_INVALID_INPUT); return -1; }
    if (!miniopus__dm_enforce_mode(d, MOGG_MODE_DATA, st)) return -1;

    if (d->state == MOGG_ST_EXPECT_HEADER || d->state == MOGG_ST_ACCUM_HEADER) {
        const int r = miniopus__dm_handle_page_header(d, input, inputLen, st, MINIOPUS_FALSE);
        if (r != 0) return (r == 2 && st->oggResult != MOGG_RES_OK) ? -1 : 0;

        const miniopus_size_t headerBytes = st->bytesConsumed;
        const miniopus_size_t remaining = (headerBytes < inputLen) ? (inputLen - headerBytes) : 0;
        if (remaining == 0) { miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA); return 0; }

        miniopus__dm_offer_body_data(d, input + headerBytes, remaining, headerBytes, st);
        return (st->oggResult == MOGG_RES_OK) ? 1 : ((st->oggResult == MOGG_RES_NEED_MORE_DATA) ? 0 : -1);
    }

    if (d->state == MOGG_ST_PROCESS_SEG) {
        miniopus__dm_offer_body_data(d, input, inputLen, 0, st);
        return (st->oggResult == MOGG_RES_OK) ? 1 : ((st->oggResult == MOGG_RES_NEED_MORE_DATA) ? 0 : -1);
    }

    miniopus__dm_set_state(st, MOGG_RES_NEED_MORE_DATA);
    return 0;
}

/* ============================================================ internal ctx
 *
 * The handle's private half, field-for-field in UMP-STMH7's shape: the demuxer
 * and its feed window, the libopus decoder, one packet of decode scratch with a
 * residual counter, the pre-skip counter, the emitted frame count, the header
 * map from the C27 open path, and the comment store. Keeping the fixed-size
 * pieces inline (feed window, page staging, scratch) is what lets a target carve
 * the whole decoder out of one pool block, which is how UMP runs it in 96 KB.
 */

typedef struct miniopus_headmap
{
    miniopus_uint64  headOff;                   /* OpusHead packet's first byte */
    miniopus_uint32  headLen;
    miniopus_uint64  tagsOff;                   /* OpusTags packet's first byte */
    miniopus_uint64  tagsPage;                  /* the page that packet starts on */
    miniopus_uint32  tagsPageLead;              /* bytes of that page's body before it */
    miniopus_uint64  tagsLen;
    miniopus_uint64  audioOff;                  /* the page the audio starts on */
    miniopus_uint32  audioSeq, audioSerial;     /* what resetForSeek has to be primed with */
} miniopus_headmap;

struct MiniOpus_Internal
{
    /* ---- io ---- */
    miniopus_read_proc  onRead;
    miniopus_seek_proc  onSeek;
    miniopus_tell_proc  onTell;
    void*             pUserData;              /* the storage handle, or the seed that carries it */
    miniopus_allocation_callbacks alloc;
    miniopus_bool32     seekable;               /* onSeek && onTell */
    miniopus_uint64     fileSize;               /* 0 = unknown */
    miniopus_uint64     ioPos;                  /* where the storage cursor actually sits */
    miniopus_uint64     seqPos;                 /* where the demuxer's next sequential read must come from */
    miniopus_bool32     cursorStale;            /* ioPos != seqPos: the next sequential read has to seek back */

    /* ---- ogg feed window (the ogg_shim chunk) ---- */
    miniopus_uint8  chunk[MINIOPUS_READ_CHUNK];
    miniopus_size_t inLen, inOff;
    miniopus_int64  chunkBase;                  /* absolute offset of chunk[0] */

    /* ---- demuxer ---- */
    struct miniopus_demux dm;
    miniopus_uint32 serial;                     /* the logical bitstream this handle plays */
    miniopus_bool32 serialSet;

    /* ---- decode state ---- */
    OpusDecoder*  pDec;
    float*        pScratch;                   /* one packet: scratchFrames * channels floats */
    miniopus_uint32 scratchFrames;
    miniopus_uint32 preSkip, preSkipLeft;
    miniopus_uint32 pktFrames, resid;           /* the residual parked in scratch */
    miniopus_int16  outputGain;                 /* OpusHead output_gain, Q8 dB, via OPUS_SET_GAIN */
    miniopus_int64  lastGranule;                /* newest page granule seen: fallback duration */
    miniopus_bool32 headersSeen, tagsSeen, eosSeen, streamEnded, seeking, mapped;
    miniopus_result lastResult;
    int           lastErrorCode;              /* micro-ogg's result code, verbatim */

    /* ---- header streaming (the open fallback) ---- */
    miniopus_uint8  hbuf[64];
    miniopus_uint32 hlen;

    /* ---- metadata ---- */
    miniopus_meta_proc onMeta;
    void*            pMetaUser;
#ifndef MINIOPUS_NO_METADATA
    miniopus_vorbis    vs;
    miniopus_bool32    pictureFound;
#endif

    /* ---- header map (C27) ---- */
    miniopus_headmap map;

    /* ---- storage mode, owned by the handle so the callbacks never dangle ---- */
    void* pFile;                              /* FILE* for miniopus_open_file */
    struct { const miniopus_uint8* pData; miniopus_size_t dataSize, cur; } mem;
};

/* --------------------------------------------------------- io primitives */

/* Sequential read: the cursor is where the last read left it, so this is one
 * fread and no seek. */
static miniopus_size_t miniopus__read_seq(struct MiniOpus* p, miniopus_uint8* dst, miniopus_size_t n)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (I->cursorStale || I->ioPos != I->seqPos) {
        if (!I->onSeek(I->pUserData, (miniopus_int64)I->seqPos, MINIOPUS_SEEK_SET)) { I->lastResult = MINIOPUS_ERROR; return 0; }
        I->ioPos = I->seqPos;
        I->cursorStale = MINIOPUS_FALSE;
    }
    const miniopus_size_t got = I->onRead(I->pUserData, dst, n);
    I->ioPos += got;
    I->seqPos += got;
    return got;
}

/* Positional read: jump, read, and leave the cursor wherever the jump landed.
 * Callers that resume sequential walking go through miniopus__page_read_at or put
 * ioPos back themselves. */
static miniopus_size_t miniopus__read_at(struct MiniOpus* p, miniopus_uint64 off, miniopus_uint8* dst, miniopus_size_t n)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (!I->seekable) return 0;
    if (I->cursorStale || I->ioPos != off) {
        if (!I->onSeek(I->pUserData, (miniopus_int64)off, MINIOPUS_SEEK_SET)) { I->lastResult = MINIOPUS_ERROR; return 0; }
    }
    const miniopus_size_t got = I->onRead(I->pUserData, dst, n);
    I->ioPos = off + got;
    /* A page walk that runs on from where it already stood costs no extra seek; only
     * one that cuts across the demuxer's place in the stream has to put it back. */
    I->cursorStale = (I->ioPos != I->seqPos) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    return got;
}

static miniopus_bool32 miniopus__seek_to(struct MiniOpus* p, miniopus_uint64 off)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (!I->seekable) return MINIOPUS_FALSE;
    if (!I->onSeek(I->pUserData, (miniopus_int64)off, MINIOPUS_SEEK_SET)) { I->lastResult = MINIOPUS_ERROR; return MINIOPUS_FALSE; }
    I->ioPos = I->seqPos = off;
    I->cursorStale = MINIOPUS_FALSE;
    return MINIOPUS_TRUE;
}

static void miniopus__find_size(struct MiniOpus* p)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (!I->seekable || !I->onTell) return;
    if (!I->onSeek(I->pUserData, 0, MINIOPUS_SEEK_END)) return;
    const miniopus_uint64 fsz = I->onTell(I->pUserData);
    if (fsz == (miniopus_uint64)-1) return;
    I->fileSize = fsz;
    I->cursorStale = MINIOPUS_TRUE;            /* the cursor is at EOF, not where the stream was */
}

/* ================================ feed window (the ogg_shim, ported)
 *
 * UMP-STMH7's ogg_shim.cpp: hold one read window, top it up when the demuxer
 * runs out of input, and remember the absolute offset of chunk[0] so a body
 * slice can say where it came from.
 */

static miniopus_bool32 miniopus__shim_refill(struct MiniOpus* p)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (I->inOff >= I->inLen) {
        I->chunkBase += (miniopus_int64)I->inLen;         /* this window is fully consumed */
        I->inOff = 0;
        I->inLen = miniopus__read_seq(p, I->chunk, MINIOPUS_READ_CHUNK);
        if (I->inLen == 0) return MINIOPUS_FALSE;        /* EOF */
    }
    return MINIOPUS_TRUE;
}

static int miniopus__handle_packet(struct MiniOpus* p, const miniopus_uint8* pkt, miniopus_size_t n);

typedef int (*miniopus__pkt_cb)(void* pUser, const miniopus_uint8* pPkt, miniopus_size_t n,
                              miniopus_bool32 isBos, miniopus_bool32 isEos, miniopus_int64 granule);
typedef int (*miniopus__body_cb)(void* pUser, const miniopus_uint8* pData, miniopus_size_t n,
                               miniopus_bool32 endOfPacket, miniopus_int64 absOff);

/* One packet, delivered to pktCb. 1 = delivered, 0 = clean end of stream,
 * -1 = container error (I->lastResult / I->lastErrorCode say which). */
static int miniopus__shim_pull(struct MiniOpus* p, miniopus__pkt_cb pktCb, void* pUser, int* outErr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (outErr) *outErr = 0;

    for (;;) {
        if (I->inOff >= I->inLen) {
            I->inOff = 0;
            I->chunkBase += (miniopus_int64)I->inLen;
            I->inLen = miniopus__read_seq(p, I->chunk, MINIOPUS_READ_CHUNK);
            if (I->inLen == 0) return 0;                /* EOF */
        }
        const miniopus_uint8* in = I->chunk + I->inOff;
        const miniopus_size_t len = I->inLen - I->inOff;
        miniopus_demux_state st;
        const int r = miniopus__dm_get_next_packet(&I->dm, in, len, &st);
        I->inOff += st.bytesConsumed;
        /* A page that ends a packet states where the timeline stands at that
         * boundary - whether the packet was handed over or skipped whole. */
        if (st.packet.is_last_on_page && st.packet.granule_position >= 0) I->lastGranule = st.packet.granule_position;

        if (r < 0) {
            I->lastResult = st.result;
            I->lastErrorCode = st.oggResult;
            if (outErr) *outErr = st.oggResult;
            return -1;
        }
        if (r == 1) {
            if (st.packet.length == 0) continue;        /* a zero-length packet carries nothing */
            if (st.packet.is_eos) I->eosSeen = MINIOPUS_TRUE;
            pktCb(pUser, st.packet.data, st.packet.length, st.packet.is_bos, st.packet.is_eos, st.packet.granule_position);
            return 1;
        }
        /* NEED_MORE_DATA or PACKET_SKIPPED: keep pulling. An oversized packet is
         * walked past without ever being buffered - which is the point. */
    }
}

/* Streaming counterpart: hand every body slice to bodyCb, skipping nothing. A
 * huge OpusTags with a cover inside is parsed this way, slice by slice.
 * 1 while more may follow, 0 at a packet boundary or EOF, -1 on error. */
static int miniopus__shim_pull_raw(struct MiniOpus* p, miniopus__body_cb bodyCb, void* pUser, int* outErr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (outErr) *outErr = 0;

    for (;;) {
        if (!miniopus__shim_refill(p)) return 0;          /* EOF */

        const miniopus_size_t inOff = I->inOff;
        const miniopus_uint8* in = I->chunk + I->inOff;
        const miniopus_size_t len = I->inLen - I->inOff;

        miniopus_demux_state st;
        const int r = miniopus__dm_get_next_data(&I->dm, in, len, &st);
        I->inOff += st.bytesConsumed;

        /* The slice handed over is the last packet.length bytes consumed. */
        const miniopus_int64 absOff = I->chunkBase + (miniopus_int64)inOff + (miniopus_int64)st.bytesConsumed
                                  - (miniopus_int64)st.packet.length;

        if (r < 0) {
            I->lastResult = st.result;
            I->lastErrorCode = st.oggResult;
            if (outErr) *outErr = st.oggResult;
            return -1;
        }
        if (r == 1) {
            if (st.packet.length > 0 && bodyCb(pUser, st.packet.data, st.packet.length, st.packet.is_end_of_packet, absOff))
                return 1;                               /* callback asked to stop */
            return 1;
        }
        /* NEED_MORE_DATA: refill the window. */
    }
}

/* Walk the feed up to the page whose granule first reaches stopGranule, decoding
 * and throwing away every packet it produces. This is a seek's approach leg: it
 * stops *at* the page boundary, so the packet that straddles it is left in
 * scratch for the discard to count frames from. 1 = anchor reached, 0 = stream
 * ended first, -1 = container error. */
static int miniopus__shim_walk_to_granule(struct MiniOpus* p, miniopus_int64 stopGranule, int* outErr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (outErr) *outErr = 0;

    for (;;) {
        if (I->lastGranule >= stopGranule) return 1;

        if (I->inOff >= I->inLen) {
            I->inOff = 0;
            I->chunkBase += (miniopus_int64)I->inLen;
            I->inLen = miniopus__read_seq(p, I->chunk, MINIOPUS_READ_CHUNK);
            if (I->inLen == 0) return 0;
        }
        const miniopus_uint8* in = I->chunk + I->inOff;
        const miniopus_size_t len = I->inLen - I->inOff;

        miniopus_demux_state st;
        const int r = miniopus__dm_get_next_packet(&I->dm, in, len, &st);
        I->inOff += st.bytesConsumed;

        if (st.packet.is_last_on_page && st.packet.granule_position >= 0) I->lastGranule = st.packet.granule_position;
        if (I->lastGranule >= stopGranule) {
            return 1;
        }

        if (r < 0) {
            I->lastResult = st.result;
            I->lastErrorCode = st.oggResult;
            if (outErr) *outErr = st.oggResult;
            return -1;
        }
        if (r == 1 && st.packet.length > 0) {
            miniopus__handle_packet(p, st.packet.data, st.packet.length);   /* decoded and dropped */
            I->resid = 0;
            I->pktFrames = 0;
        }
    }
}

/* Restart the feed at an absolute page offset, priming the demuxer's expected
 * page sequence and serial the way ogg_shim_reset_for_seek does. */
static miniopus_bool32 miniopus__shim_seek_to(struct MiniOpus* p, miniopus_uint64 pageOff, miniopus_uint32 pageSeq, miniopus_uint32 serial)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (!miniopus__seek_to(p, pageOff)) return MINIOPUS_FALSE;
    miniopus__dm_reset_for_seek(&I->dm, pageSeq, serial);
    I->inOff = 0;
    I->inLen = 0;
    I->chunkBase = (miniopus_int64)pageOff;
    return MINIOPUS_TRUE;
}
/* ========================================================= page-level reads
 *
 * Port of UMP-STMH7's ogg_pages.{h,c}: the two things a decoder needs that a
 * packet do not give it - where the next page begins, and where a packet's
 * bytes sit in the file. micro-ogg parses page headers too, but its parser is
 * private and never reports the next page's offset, and its oversized-packet
 * skip still *reads* every byte it skips; page-level walking is what lets a
 * decoder hop over a 1.6 MB cover with tens of 282-byte reads instead of
 * hundreds of thousands of body bytes (C27).
 */

typedef struct
{
    miniopus_int64  granule;                  /* -1 = this page completes no packet */
    miniopus_uint32 serial, seq;
    miniopus_uint8  flag, segs;
    miniopus_size_t headerSize, bodySize, pageSize;
} miniopus_page_hdr;

static miniopus_bool32 miniopus__page_hdr_parse(const miniopus_uint8* buf, miniopus_size_t n, miniopus_page_hdr* out)
{
    if (n < 27) return MINIOPUS_FALSE;
    if (buf[0] != 'O' || buf[1] != 'g' || buf[2] != 'g' || buf[3] != 'S') return MINIOPUS_FALSE;
    if (buf[4] != 0) return MINIOPUS_FALSE;                          /* unknown stream structure version */

    const miniopus_uint8 segs = buf[26];
    if (segs < 1) return MINIOPUS_FALSE;                             /* 0 is out of range for a page */
    if (27 + (miniopus_size_t)segs > n) return MINIOPUS_FALSE;        /* table cut short by the read */

    size_t body = 0;
    for (miniopus_uint8 i = 0; i < segs; i++) body += buf[27 + i];

    out->granule    = miniopus__le64(buf + 6);
    out->serial     = miniopus__le32(buf + 14);
    out->seq        = miniopus__le32(buf + 18);
    out->flag       = buf[5];
    out->segs       = segs;
    out->headerSize = 27 + (miniopus_size_t)segs;
    out->bodySize   = body;
    out->pageSize   = out->headerSize + body;
    return MINIOPUS_TRUE;
}

/* Read at an absolute offset without losing the sequential read cursor: the
 * excursion is undone (and the feed window is marked stale) so the demuxer can
 * carry on exactly where it left off. */
static miniopus_size_t miniopus__page_read_at(struct MiniOpus* p, miniopus_uint64 off, miniopus_uint8* dst, miniopus_size_t n)
{
    struct MiniOpus_Internal* I = p->pInternal;
    /* The demuxer's place in the stream is the feed window's end; that is what a
     * later sequential read seeks back to if this excursion cut across it. */
    const miniopus_uint64 resume = (miniopus_uint64)(I->chunkBase + (miniopus_int64)I->inLen);
    const miniopus_size_t got = miniopus__read_at(p, off, dst, n);
    I->seqPos = resume;
    I->cursorStale = (I->ioPos != I->seqPos) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    return got;
}

/* ---- the header packets, without reading the album art (C27) ---- */

#define MO_PAGE_WALK_HOPS  4096
#define MO_PAGE_SKIP_HOPS  65536
#define MO_TAGS_SLICE      512
#define MO_TAGS_SLICES     128

/* Read the page header sitting at an absolute offset and check it belongs to this
 * bitstream. A four-byte magic match inside a 1.6 MB base64 cover is not evidence
 * of a page; parsing it and comparing the serial is. */
static miniopus_bool32 miniopus__page_valid_at(struct MiniOpus* p, miniopus_uint64 off, miniopus_page_hdr* out)
{
    miniopus_uint8 hdr[MO_PAGE_HDR_MAX];
    struct MiniOpus_Internal* I = p->pInternal;
    if (miniopus__page_read_at(p, off, hdr, MO_PAGE_HDR_MAX) < 27) return MINIOPUS_FALSE;
    if (!miniopus__page_hdr_parse(hdr, MO_PAGE_HDR_MAX, out)) return MINIOPUS_FALSE;
    if (out->serial != I->serial) return MINIOPUS_FALSE;
    return MINIOPUS_TRUE;
}

static miniopus_bool32 miniopus__headmap_map(struct MiniOpus* p, miniopus_uint8* hdr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    struct miniopus_headmap* map = &I->map;
    memset(map, 0, sizeof(*map));

    miniopus_uint64 off = 0;
    int packetsDone = 0;                       /* 0 = OpusHead, 1 = OpusTags */
    int cur = -1;
    miniopus_uint32 curLen = 0;

    for (int hop = 0; hop < MO_PAGE_WALK_HOPS && off + 27 <= I->fileSize; hop++) {
        if (miniopus__page_read_at(p, off, hdr, MO_PAGE_HDR_MAX) == 0) return MINIOPUS_FALSE;

        miniopus_page_hdr ph;
        if (!miniopus__page_hdr_parse(hdr, MO_PAGE_HDR_MAX, &ph)) return MINIOPUS_FALSE;
        if (ph.serial != I->serial) { if (hop == 0) I->serial = ph.serial; else return MINIOPUS_FALSE; }

        miniopus_uint64 bo = off + ph.headerSize;

        for (miniopus_uint8 i = 0; i < ph.segs; i++) {
            const miniopus_size_t v = hdr[27 + i];
            if (cur < 0) {
                cur = packetsDone;
                curLen = 0;
                if (cur == 0) map->headOff = bo;
                if (cur == 1) {
                    map->tagsOff       = bo;
                    map->tagsPage      = off;
                    map->tagsPageLead  = (miniopus_uint32)(bo - (off + ph.headerSize));
                    return MINIOPUS_TRUE;
                }
            }
            curLen += (miniopus_uint32)v;
            if (v < 255) {
                if (cur == 0) map->headLen = curLen;
                cur = -1;
                packetsDone++;
            }
            bo += v;
        }

        if (ph.pageSize == 0) return MINIOPUS_FALSE;                /* a page that does not advance is a loop */
        off += ph.pageSize;
    }
    return MINIOPUS_FALSE;
}

/* ---- walking one packet's bytes through the page chain ---- */

typedef struct
{
    struct MiniOpus* p;
    miniopus_uint8*  hdr;
    miniopus_uint64  page;                       /* the page the cursor sits on */
    miniopus_uint64  bodyAt;                     /* file offset of the next unread body byte */
    int            segI, segs;
    size_t         segLeft;
    miniopus_bool32  pktDone;
    miniopus_uint64  nextPage;                   /* 0 = the chain ends here */
    miniopus_uint64  consumed;
    int            hops;
} miniopus_pkt;

static miniopus_uint8 miniopus__pkt_seg(const miniopus_pkt* c, int i) { return c->hdr[27 + i]; }

/* Land on `page`, skipping `lead` bytes of its body (the part belonging to
 * earlier packets), and point at the segment holding the first byte we want. */
static miniopus_bool32 miniopus__pkt_open(miniopus_pkt* c, miniopus_uint64 page, size_t lead)
{
    c->page = page;
    c->nextPage = 0;
    c->pktDone = MINIOPUS_FALSE;
    c->hops++;

    if (miniopus__page_read_at(c->p, page, c->hdr, MO_PAGE_HDR_MAX) == 0) return MINIOPUS_FALSE;

    miniopus_page_hdr ph;
    if (!miniopus__page_hdr_parse(c->hdr, MO_PAGE_HDR_MAX, &ph)) return MINIOPUS_FALSE;
    if (lead > ph.bodySize) return MINIOPUS_FALSE;

    int i = 0;
    size_t acc = 0;
    while (i < ph.segs && acc + miniopus__pkt_seg(c, i) <= lead) { acc += miniopus__pkt_seg(c, i); i++; }
    if (i == ph.segs) return MINIOPUS_FALSE;                        /* lead is past the last segment */

    c->segs = ph.segs;
    c->segI = i;
    c->segLeft = miniopus__pkt_seg(c, i) - (lead - acc);
    c->bodyAt = page + ph.headerSize + lead;
    c->nextPage = page + ph.pageSize;
    return MINIOPUS_TRUE;
}

static miniopus_bool32 miniopus__pkt_next(miniopus_pkt* c)
{
    for (;;) {
        if (c->pktDone) return MINIOPUS_FALSE;

        if (c->segI + 1 < c->segs) {
            c->segI++;
            const size_t v = miniopus__pkt_seg(c, c->segI);
            if (v == 0) continue;                                  /* an empty segment carries nothing */
            c->segLeft = v;
            return MINIOPUS_TRUE;
        }

        if (c->nextPage == 0) return MINIOPUS_FALSE;
        if (c->hops >= MO_PAGE_SKIP_HOPS) return MINIOPUS_FALSE; /* a lacing table that never runs out */
        if (!miniopus__pkt_open(c, c->nextPage, 0)) return MINIOPUS_FALSE;
        if (c->segLeft == 0) continue;
        return MINIOPUS_TRUE;
    }
}

static void miniopus__pkt_end_segment(miniopus_pkt* c)
{
    if (c->segLeft == 0 && miniopus__pkt_seg(c, c->segI) < 255) c->pktDone = MINIOPUS_TRUE;
}

/* Copy up to n packet bytes, stopping at a segment boundary so the run is
 * contiguous in the file. *fileOff is where that run starts. */
static size_t miniopus__pkt_take(miniopus_pkt* c, miniopus_uint8* dst, size_t n, miniopus_uint64* fileOff)
{
    size_t total = 0;
    *fileOff = c->bodyAt;

    /* Segments of one page are contiguous in the file, so a run continues across
     * them without another header read; only a page hop costs one. Bounded by the
     * packet's own lacing: a segment under 255 ends the packet and the run. */
    while (total < n) {
        while (c->segLeft == 0) {
            if (c->pktDone) return total;
            if (c->segI + 1 < c->segs) {
                c->segI++;
                const size_t v = miniopus__pkt_seg(c, c->segI);
                if (v == 0) { miniopus__pkt_end_segment(c); continue; }
                c->segLeft = v;
                continue;
            }
            if (!miniopus__pkt_next(c)) return total;
        }

        const size_t k = (n - total < c->segLeft) ? (n - total) : c->segLeft;
        const size_t got = miniopus__page_read_at(c->p, c->bodyAt, dst + total, k);
        if (got == 0) break;
        c->bodyAt += got;
        c->segLeft -= got;
        c->consumed += got;
        total += got;
        miniopus__pkt_end_segment(c);
        if (c->pktDone) break;
    }
    return total;
}

/* Move n packet bytes forward without reading them: one header read per page
 * hop, which is what makes skipping a 1.6 MB picture cost tens of reads. */
static miniopus_bool32 miniopus__pkt_advance(miniopus_pkt* c, size_t n)
{
    while (n) {
        while (c->segLeft == 0) if (!miniopus__pkt_next(c)) return MINIOPUS_FALSE;
        const size_t k = (n < c->segLeft) ? n : c->segLeft;
        c->bodyAt += k;
        c->segLeft -= k;
        c->consumed += k;
        n -= k;
        miniopus__pkt_end_segment(c);
    }
    return MINIOPUS_TRUE;
}

/* Run the cursor to the end of the packet reading nothing more. */
static miniopus_bool32 miniopus__pkt_finish(miniopus_pkt* c)
{
    for (int hop = 0; hop < MO_PAGE_WALK_HOPS; hop++) {
        if (c->pktDone) return MINIOPUS_TRUE;
        const miniopus_uint8 v = miniopus__pkt_seg(c, c->segI);
        c->bodyAt += c->segLeft;
        c->consumed += c->segLeft;
        c->segLeft = 0;
        if (v < 255) { c->pktDone = MINIOPUS_TRUE; return MINIOPUS_TRUE; }
        if (!miniopus__pkt_next(c)) return MINIOPUS_FALSE;
    }
    return MINIOPUS_FALSE;
}

/* =============================================================== granule math
 *
 * UMP-STMH7's model, kept exactly: a page's granule position is the end time of
 * the last packet that completes on it, so
 *
 *     total PCM frames  =  EOS page granule  -  pre_skip
 *
 * read off the last page at open (the tail scan below), and, if that scan finds
 * nothing, from the newest granule the decode happens to pass (a raw stream with
 * no EOS page). A seek asks for the page with the greatest granule strictly
 * below target_granule = pcm + pre_skip, reinitialises the demuxer and the
 * decoder there, then decodes and throws away (target_granule - page granule)
 * frames - at most one page of audio.
 */

static void miniopus__find_total(struct MiniOpus* p)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (!I->seekable || I->fileSize == 0) return;

    miniopus_uint8 chunk[MINIOPUS_READ_CHUNK];
    miniopus_uint64 off = (I->fileSize > sizeof(chunk)) ? I->fileSize - sizeof(chunk) : 0;

    while (off < I->fileSize && p->totalPCMFrameCount == 0) {
        const miniopus_size_t n = miniopus__page_read_at(p, off, chunk, sizeof(chunk));
        if (n < 27) break;

        /* Scan backwards in this chunk for a valid OggS page carrying the EOS flag. */
        for (miniopus_int64 i = (miniopus_int64)n - 27; i >= 0; i--) {
            if (chunk[i] != 'O' || memcmp(chunk + i, "OggS", 4) != 0 || chunk[i + 4] != 0) continue;
            if (!(chunk[i + 5] & 0x04)) continue;
            const miniopus_int64 gran = miniopus__le64(chunk + i + 6);
            if (gran > 0 && I->preSkip < (miniopus_uint64)gran)
                p->totalPCMFrameCount = (miniopus_uint64)(gran - (miniopus_int64)I->preSkip);
            break;
        }

        if (off == 0) break;
        off = (off > sizeof(chunk)) ? off - sizeof(chunk) : 0;
    }
}

#ifndef MINIOPUS_NO_SEEKING

/* ---- granule bisection (port of UMP-STMH7's opus_bisect_*) ---- */

/* `known`, when given, is this page's header already parsed by the caller (the
 * window scans read it to validate the hit, and re-reading it here was a third
 * fetch of the same 282 bytes per bisection step). */
static miniopus_bool32 miniopus__bisect_granulate(struct MiniOpus* p, miniopus_uint64 offset, miniopus_uint8* scratch,
                                              miniopus_uint64* outOff, miniopus_int64* outGranule,
                                              const miniopus_page_hdr* known, miniopus_page_hdr* outHdr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus_uint64 off = offset;

    for (int hop = 0; hop < MO_BISECT_GRAN_HOPS; hop++) {
        if (off + 27 > I->fileSize) return MINIOPUS_FALSE;

        miniopus_page_hdr ph;
        if (hop == 0 && known) {
            ph = *known;
        } else {
            if (miniopus__page_read_at(p, off, scratch, MO_PAGE_HDR_MAX) == 0) return MINIOPUS_FALSE;
            if (!miniopus__page_hdr_parse(scratch, MO_PAGE_HDR_MAX, &ph)) return MINIOPUS_FALSE;
            if (ph.serial != I->serial) return MINIOPUS_FALSE;
        }
        if (ph.granule >= 0) { *outOff = off; *outGranule = ph.granule; if (outHdr) *outHdr = ph; return MINIOPUS_TRUE; }
        off += ph.pageSize;                                        /* granule -1: one hop, one header read */
    }
    return MINIOPUS_FALSE;
}

static miniopus_uint64 miniopus__bisect_scan_bwd(struct MiniOpus* p, miniopus_uint64 startOffset, miniopus_uint8* scratch,
                                             miniopus_size_t scratchSize, miniopus_page_hdr* outHdr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (startOffset >= I->fileSize) startOffset = I->fileSize - 1;
    if (startOffset < 27) return (miniopus_uint64)-1;

    miniopus_uint64 cur = startOffset;
    miniopus_uint64 scanned = 0;

    while (cur >= 27 && scanned < (miniopus_uint64)MO_BISECT_SCAN_MAX) {
        const miniopus_uint64 chunkStart = (cur > scratchSize) ? cur - scratchSize : 0;
        const miniopus_size_t chunkLen = (miniopus_size_t)(cur - chunkStart);
        if (chunkLen < 27) break;

        const miniopus_size_t got = miniopus__page_read_at(p, chunkStart, scratch, chunkLen);
        if (got < 27) break;
        for (miniopus_int64 i = (miniopus_int64)got - 4; i >= 0; i--) {
            if (scratch[i] != 'O' || scratch[i + 1] != 'g' || scratch[i + 2] != 'g' || scratch[i + 3] != 'S' || scratch[i + 4] != 0) continue;
            const miniopus_uint8 segs = scratch[i + 26];
            if (segs < 1) continue;
            miniopus_page_hdr ph;
            if ((miniopus_size_t)(i + 27 + segs) <= got) {
                /* The whole lacing table is already in hand: compare the serial here,
                   for free. A four-byte magic match inside a 1.6 MB base64 cover is not
                   evidence of a page, but magic plus this serial is. */
                if ((miniopus_uint32)miniopus__le32(scratch + i + 14) != I->serial) continue;
                miniopus__page_hdr_parse(scratch + i, got - (miniopus_size_t)i, &ph);
            } else if (!miniopus__page_valid_at(p, chunkStart + (miniopus_uint64)i, &ph)) {
                /* A page starting near the window's top edge has its table outside the
                   window, which is not a reason to miss it: read it where it actually sits. */
                continue;
            }
            if (outHdr) *outHdr = ph;
            return chunkStart + (miniopus_uint64)i;
        }
        scanned += cur - chunkStart;
        cur = chunkStart;
    }
    return (miniopus_uint64)-1;
}

static miniopus_uint64 miniopus__bisect_scan_fwd(struct MiniOpus* p, miniopus_uint64 startOffset, miniopus_uint8* scratch,
                                             miniopus_size_t scratchSize, miniopus_page_hdr* outHdr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (startOffset + 27 > I->fileSize) return (miniopus_uint64)-1;

    miniopus_uint64 cur = startOffset;
    miniopus_uint64 scanned = 0;

    while (cur + 27 <= I->fileSize && scanned < (miniopus_uint64)MO_BISECT_SCAN_MAX) {
        miniopus_size_t chunkLen = scratchSize;
        if (cur + chunkLen > I->fileSize) chunkLen = (miniopus_size_t)(I->fileSize - cur);
        if (chunkLen < 27) break;

        const miniopus_size_t got = miniopus__page_read_at(p, cur, scratch, chunkLen);
        if (got < 27) break;

        for (miniopus_size_t i = 0; i + 4 <= got; i++) {
            if (scratch[i] != 'O' || scratch[i + 1] != 'g' || scratch[i + 2] != 'g' || scratch[i + 3] != 'S' || scratch[i + 4] != 0) continue;
            const miniopus_uint8 segs = scratch[i + 26];
            if (segs < 1) continue;
            miniopus_page_hdr ph;
            if (i + 27 + (miniopus_size_t)segs <= got) {
                if ((miniopus_uint32)miniopus__le32(scratch + i + 14) != I->serial) continue;
                miniopus__page_hdr_parse(scratch + i, got - i, &ph);
            } else if (!miniopus__page_valid_at(p, cur + i, &ph)) continue;
            if (outHdr) *outHdr = ph;
            return cur + i;
        }
        scanned += got;
        cur += got;
    }
    return (miniopus_uint64)-1;
}

static void miniopus__bisect_refine_back(struct MiniOpus* p, miniopus_uint64 loOff, miniopus_int64 loG,
                                       miniopus_uint64 hiOff, miniopus_int64 hiG, miniopus_int64 targetGranule,
                                       miniopus_uint8* scratch, miniopus_size_t scratchSize,
                                       miniopus_uint64* outOff, miniopus_int64* outGranule, miniopus_page_hdr* outHdr)
{
    miniopus_uint64 result = loOff;
    miniopus_int64 resultG = loG;
    miniopus_page_hdr resultHdr;
    miniopus_bool32 haveHdr = MINIOPUS_FALSE;

    if (hiG >= 0 && hiG < targetGranule) {
        *outOff = hiOff; *outGranule = hiG;
        if (outHdr && miniopus__page_read_at(p, hiOff, scratch, MO_PAGE_HDR_MAX) >= 27 &&
            miniopus__page_hdr_parse(scratch, MO_PAGE_HDR_MAX, outHdr)) { /* header reported */ }
        return;
    }

    miniopus_uint64 cur = hiOff;
    int hops = 0;
    while (cur > loOff && hops < MO_BISECT_REFINE) {
        miniopus_page_hdr ph;
        miniopus_uint64 prev = miniopus__bisect_scan_bwd(p, cur - 1, scratch, scratchSize, &ph);
        if (prev == (miniopus_uint64)-1 || prev <= loOff) break;

        cur = prev;
        hops++;
        if (ph.granule >= 0 && ph.granule < targetGranule) {
            result = prev; resultG = ph.granule; resultHdr = ph; haveHdr = MINIOPUS_TRUE;
            break;
        }
    }
    *outOff = result; *outGranule = resultG;
    if (outHdr) {
        if (haveHdr) *outHdr = resultHdr;
        else if (miniopus__page_read_at(p, result, scratch, MO_PAGE_HDR_MAX) >= 27)
            miniopus__page_hdr_parse(scratch, MO_PAGE_HDR_MAX, outHdr);
    }
}

/* Binary search for the page whose granule is the greatest < targetGranule.
 * Returns the page's byte offset, or (miniopus_uint64)-1. Two shapes have to
 * survive, both recorded by UMP-STMH7 on its bench library:
 *
 *  - Pages whose granule is -1. A muxer that lets a 255-segment page continue a
 *    packet completes nothing on it, so a run of them carries no position at all.
 *    Reading -1 as a failed read (which is what broke the original search) makes
 *    the caller fall back to walking back from EOF a page at a time - 182 hops and
 *    ~660 storage round trips on one track, enough to freeze the UI on target -
 *    and a fallback that stops on a -1 page hands back a granule of -1, turning
 *    `target - seek_granule` into `target + 1` discarded frames and the seek into a
 *    failed track. granulate() steps over them with one header read per hop.
 *  - An interval whose interpolated midpoint lands before the first page start
 *    inside it. scan_bwd then returns lo_off, which is not progress, so the search
 *    asks scan_fwd instead.
 */
static miniopus_uint64 miniopus__bisect_seek_page(struct MiniOpus* p, miniopus_int64 targetGranule, miniopus_int64* outGranule,
                                              miniopus_uint8* scratch, miniopus_size_t scratchSize, miniopus_page_hdr* outLanding)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (I->fileSize == 0) return (miniopus_uint64)-1;

    miniopus_uint64 loOff; miniopus_int64 loG; miniopus_page_hdr loHdr;
    if (!miniopus__bisect_granulate(p, 0, scratch, &loOff, &loG, NULL, &loHdr)) return (miniopus_uint64)-1;

    miniopus_page_hdr hiHdr;
    miniopus_uint64 hiOff = miniopus__bisect_scan_bwd(p, I->fileSize - 1, scratch, scratchSize, &hiHdr);
    if (hiOff == (miniopus_uint64)-1) return (miniopus_uint64)-1;
    miniopus_int64 hiG = hiHdr.granule;

    if (targetGranule <= loG) { *outGranule = loG; if (outLanding) *outLanding = loHdr; return loOff; }
    if (hiG >= 0 && targetGranule > hiG) { *outGranule = hiG; if (outLanding) *outLanding = hiHdr; return hiOff; }

    miniopus_page_hdr resultHdr = loHdr;
    miniopus_uint64 result = loOff;
    miniopus_int64 resultG = loG;

    miniopus_int64 iter = 0;
    while (hiOff - loOff > 1 && iter < MO_BISECT_ITERS) {
        double ratio = (hiG > loG) ? (double)(targetGranule - loG) / (double)(hiG - loG) : 0.5;
        if (ratio < 0.005) ratio = 0.005;
        if (ratio > 0.995) ratio = 0.995;
        const miniopus_uint64 guess = loOff + (miniopus_uint64)((double)(hiOff - loOff) * ratio);

        miniopus_page_hdr midHdr;
        miniopus_bool32 haveMid = MINIOPUS_FALSE;
        miniopus_uint64 page = miniopus__bisect_scan_bwd(p, guess, scratch, scratchSize, &midHdr);
        if (page == (miniopus_uint64)-1 || page <= loOff)
            page = miniopus__bisect_scan_fwd(p, guess, scratch, scratchSize, &midHdr);
        if (page == (miniopus_uint64)-1 || page <= loOff || page >= hiOff) break;
        haveMid = MINIOPUS_TRUE;

        miniopus_uint64 mOff; miniopus_int64 mg; miniopus_page_hdr mHdr;
        if (!miniopus__bisect_granulate(p, page, scratch, &mOff, &mg, haveMid ? &midHdr : NULL, &mHdr)) break;
        if (mOff <= loOff || mOff >= hiOff) break;

        if (mg < targetGranule) { loOff = mOff; loG = mg; result = mOff; resultG = mg; resultHdr = mHdr; }
        else                    { hiOff = mOff; hiG = mg; }
        iter++;
    }

    /* refine_back walks back from hi when the bisection stopped with a page on each
     * side that is not the closest one below the target. */
    if (resultG == hiG || result == loOff) {
        miniopus__bisect_refine_back(p, loOff, loG, hiOff, hiG, targetGranule, scratch, scratchSize,
                                   &result, &resultG, &resultHdr);
        *outGranule = resultG;
    } else {
        *outGranule = resultG;
        if (miniopus__page_read_at(p, result, scratch, MO_PAGE_HDR_MAX) >= 27)
            miniopus__page_hdr_parse(scratch, MO_PAGE_HDR_MAX, &resultHdr);
    }

    /* A landing page has to be a page the demuxer can restart on *and* finish: one
     * whose first segment continues the previous page's packet cannot be restarted on
     * (after resetForSeek there is no in-flight packet to resume, so the fragment
     * reaches the decoder as if it were a whole packet), and one whose last segment is
     * 255 hands a packet onward that the next page then shows only the tail of. Either
     * way that packet's frames never get decoded while its samples stay counted in the
     * granules, so the discard - which counts frames - overshoots the target by exactly
     * that much. Step back to the nearest page that both opens and closes a packet;
     * ogg_headmap_scan_tags applies the same rule when it reports where audio starts. */
    for (int hops = 0; hops < MO_BISECT_GRAN_HOPS && result > 0; hops++) {
        if (!(resultHdr.flag & 0x01) && resultHdr.segs > 0 && scratch[27 + resultHdr.segs - 1] < 255) break;
#ifdef MINIOPUS_TRACE
        printf("[trace] landing page %llu opens or closes mid-packet, stepping back\n", (unsigned long long)result);
#endif
        miniopus_page_hdr pp;
        const miniopus_uint64 prev = miniopus__bisect_scan_bwd(p, result - 1, scratch, scratchSize, &pp);
        if (prev == (miniopus_uint64)-1 || prev >= result || pp.serial != I->serial) break;
        result = prev;
        resultHdr = pp;
        if (pp.granule >= 0 && pp.granule < targetGranule) *outGranule = pp.granule;
    }

    if (outLanding) *outLanding = resultHdr;
    return result;
}

#endif /* !MINIOPUS_NO_SEEKING */

#ifndef MINIOPUS_NO_METADATA

/* Deliver the parsed comment header through the user callback. */
static void miniopus__deliver_meta(struct MiniOpus* p, miniopus_vorbis* vs)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (vs->stage != MOVS_STAGE_DONE) return;
    if (vs->picFound) I->pictureFound = MINIOPUS_TRUE;      /* miniopus_has_picture works without a callback */
    if (!I->onMeta) return;

    if (vs->picFound) {
        miniopus_metadata md; memset(&md, 0, sizeof(md));
        md.type = MINIOPUS_METADATA_PICTURE;
        miniopus_uint8* pImage = NULL; miniopus_uint64 imageSize = 0;
        if (vs->pPicVal) {
            miniopus_size_t rawCap = (vs->picValLen / 4 + 1) * 3;
            if (rawCap > (miniopus_size_t)MINIOPUS_MAX_PICTURE_BYTES) rawCap = (miniopus_size_t)MINIOPUS_MAX_PICTURE_BYTES;
            pImage = (miniopus_uint8*)miniopus__malloc(&I->alloc, rawCap ? rawCap : 1);
            if (pImage) {
                miniopus_size_t rn = miniopus__b64decode((const char*)vs->pPicVal, vs->picValLen, pImage, rawCap);
                imageSize = (vs->picDataLenKnown && vs->picDataLen <= rn) ? vs->picDataLen : rn;
                if (!vs->picDataLenKnown) { miniopus__free(&I->alloc, pImage); pImage = NULL; imageSize = 0; }
            }
        }
        md.data.picture.pictureType = vs->picType;
        md.data.picture.mimeType    = vs->picMime[0] ? vs->picMime : "unknown";
        md.data.picture.description = vs->picDesc;
        md.data.picture.width       = vs->picW;
        md.data.picture.height      = vs->picH;
        md.data.picture.depth       = vs->picDepth;
        md.data.picture.colors      = vs->picColors;
        md.data.picture.pData       = pImage;
        md.data.picture.dataSize    = imageSize ? imageSize : (vs->picDataLenKnown ? vs->picDataLen : 0);
        I->onMeta(I->pMetaUser, &md);
        if (pImage) miniopus__free(&I->alloc, pImage);
    }

    miniopus_metadata md; memset(&md, 0, sizeof(md));
    md.type = MINIOPUS_METADATA_TAGS;
    md.data.tags.vendorString = vs->vendor;
    md.data.tags.tagCount     = vs->tagsStored;
    md.data.tags.oversizedCommentCount = vs->oversizedCount + vs->storeDropped;
    miniopus_tag tags[64];
    miniopus_uint32 n = vs->tagsStored > 64 ? 64 : vs->tagsStored;
    miniopus_size_t off = 0;
    for (miniopus_uint32 i = 0; i < n && off < vs->storeLen; i++) {
        tags[i].key = (const char*)(vs->pStore + off);
        off += strlen(tags[i].key) + 1;
        tags[i].value = (const char*)(vs->pStore + off);
        off += strlen(tags[i].value) + 1;
    }
    md.data.tags.pTags = tags;
    I->onMeta(I->pMetaUser, &md);
}
#endif /* !MINIOPUS_NO_METADATA */

/* ---- ogg_headmap_scan_tags (C27), ported --------------------------------
 *
 * Read the head of the tags packet - vendor length, comment count, and the
 * length of every comment up to and including the album art - then hop the art
 * blob without reading it, then read what sits after it. Every comment's length
 * is declared before the comment, so the bytes that remain after the comments
 * the player wants are bytes nothing here needs.
 *
 * Fails (so the caller can fall back to streaming the packet) when the packet's
 * comment lengths disagree with the extent its page lacing declared, when the
 * comments do not finish inside the slice budget, or when the first audio packet
 * starts partway into a page - a page whose first segment continues the previous
 * page's packet is nowhere a demuxer can restart.
 */
/* The tags packet is over, so the page after it is where the audio starts - unless
 * another packet began inside the same page, which is nowhere a demuxer can restart. */
static miniopus_bool32 miniopus__pkt_audio_after(miniopus_pkt* c, miniopus_headmap* map)
{
    struct MiniOpus_Internal* I = c->p->pInternal;
    if (!miniopus__pkt_finish(c)) return MINIOPUS_FALSE;                     /* the comments ran past the packet */
    map->tagsLen = c->consumed;

    if (c->segI + 1 < c->segs) return MINIOPUS_FALSE;
    if (c->nextPage == 0) return MINIOPUS_FALSE;
    if (!miniopus__pkt_open(c, c->nextPage, 0)) return MINIOPUS_FALSE;

    miniopus_page_hdr ah;
    if (!miniopus__page_hdr_parse(c->hdr, MO_PAGE_HDR_MAX, &ah)) return MINIOPUS_FALSE;
    if (ah.flag & 0x01) return MINIOPUS_FALSE;                            /* a continuation: no packet boundary to start from */
    if (ah.serial != I->serial) return MINIOPUS_FALSE;

    map->audioOff    = c->page;
    map->audioSeq    = ah.seq;
    map->audioSerial = ah.serial;
    return MINIOPUS_TRUE;
}

#ifndef MINIOPUS_NO_METADATA

static miniopus_bool32 miniopus__headmap_scan_tags(struct MiniOpus* p, miniopus_uint8* hdr, miniopus_vorbis* vs)
{
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus_uint8 slice[MO_TAGS_SLICE];

    miniopus_pkt c;
    memset(&c, 0, sizeof(c));
    c.p = p; c.hdr = hdr;

    if (!miniopus__pkt_open(&c, I->map.tagsPage, I->map.tagsPageLead)) return MINIOPUS_FALSE;

    miniopus__vs_reset(vs, NULL, NULL, 0);
    if (!miniopus__vs_ensure_buf(vs, &I->alloc)) return MINIOPUS_FALSE;

    miniopus_bool32 parserDone = MINIOPUS_FALSE;
    for (int i = 0; i < MO_TAGS_SLICES; i++) {
        miniopus_uint64 fileOff = 0;
        const size_t got = miniopus__pkt_take(&c, slice, MO_TAGS_SLICE, &fileOff);
        if (got == 0) break;                                       /* the packet's lacing ended */

        (void)fileOff;
        if (miniopus__vs_feed(vs, slice, got, &I->alloc)) { parserDone = MINIOPUS_TRUE; break; }

        /* The parser has just counted a comment too long to look at - the album
         * art. Moving the cursor past it instead of reading it is the whole point
         * of this path. */
        const miniopus_uint32 skip = miniopus__vs_pending_skip(vs);
        if (skip) {
            miniopus__vs_skip(vs, skip);
            if (!miniopus__pkt_advance(&c, skip)) return MINIOPUS_FALSE;
        }

        if (miniopus__vs_complete(vs)) { parserDone = MINIOPUS_TRUE; break; }
    }

    if (!parserDone && !miniopus__vs_complete(vs)) return MINIOPUS_FALSE;
    return miniopus__pkt_audio_after(&c, &I->map);
}

/* The find-mode tags walk: same page-chain cursor, parser stops on the key. */
static miniopus_bool32 miniopus__headmap_scan_tags_find(struct MiniOpus* p, miniopus_uint8* hdr, miniopus_vorbis* vs)
{
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus_uint8 slice[MO_TAGS_SLICE];

    miniopus_pkt c;
    memset(&c, 0, sizeof(c));
    c.p = p; c.hdr = hdr;

    if (!miniopus__pkt_open(&c, I->map.tagsPage, I->map.tagsPageLead)) return MINIOPUS_FALSE;
    if (!miniopus__vs_ensure_buf(vs, &I->alloc)) return MINIOPUS_FALSE;

    for (int i = 0; i < MO_TAGS_SLICES; i++) {
        miniopus_uint64 fileOff = 0;
        const size_t got = miniopus__pkt_take(&c, slice, MO_TAGS_SLICE, &fileOff);
        if (got == 0) break;
        if (miniopus__vs_feed(vs, slice, got, &I->alloc)) return vs->findHit ? MINIOPUS_TRUE : MINIOPUS_FALSE;

        const miniopus_uint32 skip = miniopus__vs_pending_skip(vs);
        if (skip) { miniopus__vs_skip(vs, skip); if (!miniopus__pkt_advance(&c, skip)) return MINIOPUS_FALSE; }
        if (miniopus__vs_complete(vs)) break;
    }
    return vs->findHit ? MINIOPUS_TRUE : MINIOPUS_FALSE;
}


#else  /* MINIOPUS_NO_METADATA: the comment header is only a place in the file. Walk its
     * lacing to find where the audio starts - which is the same page-level cursor, just
     * without the parser, and it reads no comment bytes either way. */
static miniopus_bool32 miniopus__headmap_scan_tags(struct MiniOpus* p, miniopus_uint8* hdr)
{
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus_pkt c;
    memset(&c, 0, sizeof(c));
    c.p = p; c.hdr = hdr;
    if (!miniopus__pkt_open(&c, I->map.tagsPage, I->map.tagsPageLead)) return MINIOPUS_FALSE;
    return miniopus__pkt_audio_after(&c, &I->map);
}

#endif /* !MINIOPUS_NO_METADATA */

#ifndef MINIOPUS_NO_METADATA

MINIOPUS_API miniopus_bool32 miniopus_find_tag(MiniOpus* pOpus, const char* pKey, char* pValueOut, miniopus_size_t valueBufferSize)
{
    if (!pOpus || !pOpus->pInternal || !pKey || !pValueOut || valueBufferSize == 0) return MINIOPUS_FALSE;
    struct MiniOpus_Internal* I = pOpus->pInternal;
    if (!I->seekable || I->fileSize == 0) { I->lastResult = MINIOPUS_INVALID_STATE; return MINIOPUS_FALSE; }

    const miniopus_uint64 resume = (miniopus_uint64)(I->chunkBase + (miniopus_int64)I->inLen);
    miniopus_uint8 hdr[MO_PAGE_HDR_MAX];
    miniopus_bool32 found = MINIOPUS_FALSE;

    miniopus_vorbis fv;
    memset(&fv, 0, sizeof(fv));
    miniopus__vs_reset(&fv, pKey, pValueOut, valueBufferSize);
    if (miniopus__vs_ensure_buf(&fv, &I->alloc)) {
        /* The mapped open already walked the page chain and knows where the comment
         * header sits; re-walking it here would double the reads for nothing. */
        if ((I->mapped && I->map.tagsPage != 0) || miniopus__headmap_map(pOpus, hdr)) {
            if (miniopus__headmap_scan_tags_find(pOpus, hdr, &fv)) found = fv.findHit;
        }
        miniopus__vs_free(&fv, &I->alloc);
    }

    /* Put the sequential reader back where the stream left it. The demuxer's own
     * state is untouched by a page-level walk; only the feed window moved. */
    I->seqPos = resume;
    I->cursorStale = MINIOPUS_TRUE;
    if (!found) I->lastResult = MINIOPUS_OK;
    return found;
}

#endif /* !MINIOPUS_NO_METADATA */

/* ============================================================ decode engine
 *
 * Port of UMP-STMH7's opus.c packet handler and read_frames loop: one packet at
 * a time through libopus, the pre-skip counted off the head of the decoded
 * stream (not off the granules), one packet parked in scratch as a residual,
 * and a bounded pull guard so a stream that keeps delivering packets but never
 * produces frames reports a failure instead of spinning.
 */

/* OpusHead (RFC 7845 5.1). Channels are limited to 1 or 2 exactly as UMP-STMH7's
 * C6 clamp does: its scratch holds two channels of OPUS_MAX_FRAME, and the
 * player's DMA path is stereo-only, so a surround file is refused at open
 * rather than overflowing scratch. */
static miniopus_bool32 miniopus__parse_head(struct MiniOpus* p, const miniopus_uint8* d, miniopus_size_t n)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (n < 19 || memcmp(d, "OpusHead", 8) != 0) { I->lastResult = MINIOPUS_INVALID_FORMAT; return MINIOPUS_FALSE; }
    if (d[8] > 15) { I->lastResult = MINIOPUS_INVALID_FORMAT; return MINIOPUS_FALSE; }   /* major version 0: accept 0..15 like opusfile - lavf-muxed files in the wild carry version 1 */
    const miniopus_uint32 ch = d[9];
    const miniopus_uint8 fam = d[18];
    if (ch < 1 || ch > 2) { I->lastResult = MINIOPUS_UNSUPPORTED; return MINIOPUS_FALSE; }   /* multistream territory */
    if (fam != 0)         { I->lastResult = MINIOPUS_UNSUPPORTED; return MINIOPUS_FALSE; }

    p->channels            = ch;
    I->preSkip             = (miniopus_uint32)miniopus__le16(d + 10);
    p->sourceSampleRate    = miniopus__le32(d + 12);
    if (p->sourceSampleRate == 0) p->sourceSampleRate = 48000;
    p->sampleRate          = 48000;                      /* granules are 48 kHz units; decode at 48 kHz */
    I->outputGain          = miniopus__le16s(d + 16);
#ifdef MINIOPUS_TRACE
    printf("[trace] parse_head: n=%zu ch=%u preSkip=%u rate=%u gain=%d fam=%u\n",
           n, ch, I->preSkip, p->sourceSampleRate, (int)I->outputGain, fam);
#endif
    return MINIOPUS_TRUE;
}

/* Grow the decode scratch so a legal 120 ms packet is decoded instead of
 * dropped. UMP-STMH7 clamps the frame_size it hands opus_decode_float (its C6)
 * because its slot is a fixed 96 KB; MiniOpus keeps UMP's 2880-frame default
 * (23 KB of scratch, the same footprint) and grows only when a file actually
 * carries a longer frame. */
static miniopus_bool32 miniopus__ensure_scratch(struct MiniOpus* p, miniopus_uint32 frames)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (frames <= MO_MAX_DECODE_FRAMES && frames > I->scratchFrames) {
        float* s = (float*)miniopus__realloc(&I->alloc, I->pScratch, (miniopus_size_t)frames * p->channels * sizeof(float));
        if (!s) return MINIOPUS_FALSE;
        I->pScratch = s;
        I->scratchFrames = frames;
    }
    return (frames <= I->scratchFrames) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
}

/* One packet: headers first, then opus_decode. Returns non-zero when the packet
 * produced a residual, 0 when it produced nothing (a header, a skip, an error). */
static int miniopus__handle_packet(struct MiniOpus* p, const miniopus_uint8* pkt, miniopus_size_t n)
{
    struct MiniOpus_Internal* I = p->pInternal;

    if (!I->headersSeen) {
        if (n >= 8 && memcmp(pkt, "OpusHead", 8) == 0) {
            miniopus__parse_head(p, pkt, n);
            I->headersSeen = MINIOPUS_TRUE;
        }
        return 0;
    }

    /* The comment header. The mapped open path already streamed it (tagsSeen);
     * re-parsing it here would re-read the same bytes and double every tag. */
    if (n >= 8 && memcmp(pkt, "OpusTags", 8) == 0) {
#ifndef MINIOPUS_NO_METADATA
        if (!I->tagsSeen) {                      /* the streamed open path has not seen it yet */
            miniopus__vs_reset(&I->vs, NULL, NULL, 0);
            miniopus__vs_ensure_buf(&I->vs, &I->alloc);
            miniopus__vs_feed(&I->vs, pkt, n, &I->alloc);
            miniopus__deliver_meta(p, &I->vs);
            I->tagsSeen = MINIOPUS_TRUE;
        }
#endif
        return 0;
    }

    /* Fallback duration: the newest page granule seen, if the tail scan found no
     * EOS page (a raw stream). Never clobbers a total already read off the last page. */
    if (p->totalPCMFrameCount == 0 && I->lastGranule > 0 && I->preSkip < (miniopus_uint64)I->lastGranule)
        p->totalPCMFrameCount = (miniopus_uint64)(I->lastGranule - (miniopus_int64)I->preSkip);

    if (I->resid) return 1;

    int samples = opus_decoder_get_nb_samples(I->pDec, pkt, (opus_int32)n);
    if (samples <= 0) {
        return 0;                                   /* corrupt frame: skipped */
    }
    if (!miniopus__ensure_scratch(p, (miniopus_uint32)samples)) { I->lastResult = MINIOPUS_OUT_OF_MEMORY; return 0; }

    int ns = opus_decode_float(I->pDec, pkt, (opus_int32)n, I->pScratch, samples, 0);
    if (ns <= 0) {
#ifdef MINIOPUS_TRACE
        printf("[trace] DECODE FAIL ns=%d len=%zu samples=%d granule=%lld\n", ns, n, samples, (long long)I->lastGranule);
#endif
        return 0;                                        /* decode error / PLC */
    }

    miniopus_uint32 frames = (miniopus_uint32)ns;
    if (I->preSkipLeft) {
        const miniopus_uint32 drop = (frames < I->preSkipLeft) ? frames : I->preSkipLeft;
        if (drop >= frames) { I->preSkipLeft -= frames; return 0; }   /* the whole packet is pre-skip */
        frames -= drop;
        I->preSkipLeft -= drop;
        /* C1: scratch is a float*, so the shift moves frames in FLOAT units. The
         * old sizeof(int16_t) here moved half the bytes and corrupted the tail of
         * every track with pre_skip > 0. */
        memmove(I->pScratch, I->pScratch + (miniopus_size_t)drop * p->channels,
                (miniopus_size_t)frames * p->channels * sizeof(*I->pScratch));
    }

    I->pktFrames = frames;
    I->resid = frames;
    return 1;
}

static int miniopus__deliver_pkt(void* pUser, const miniopus_uint8* pkt, miniopus_size_t n,
                          miniopus_bool32 isBos, miniopus_bool32 isEos, miniopus_int64 granule)
{
    struct MiniOpus* p = (struct MiniOpus*)pUser;
    (void)isBos;
    if (isEos) p->pInternal->eosSeen = MINIOPUS_TRUE;
    if (granule >= 0) p->pInternal->lastGranule = granule;
    return miniopus__handle_packet(p, pkt, n);
}

/* Serve frames: drain the residual first, then pull packets. */
static miniopus_uint64 miniopus__read_frames(struct MiniOpus* p, miniopus_uint64 framesToRead, float* pFramesOut)
{
    struct MiniOpus_Internal* I = p->pInternal;
    if (!I->pDec || I->streamEnded) return 0;

    miniopus_uint64 written = 0;
    miniopus_uint32 guard = 0;
    int shimRc = 1;

    while (written < framesToRead) {
        if (I->resid) {
            miniopus_uint32 served = I->pktFrames - I->resid;
            miniopus_uint32 m = I->resid;
            if ((miniopus_uint64)m > framesToRead - written) m = (miniopus_uint32)(framesToRead - written);
            memcpy(pFramesOut + written * p->channels, I->pScratch + (miniopus_size_t)served * p->channels,
                   (miniopus_size_t)m * p->channels * sizeof(float));
            I->resid -= m;
            written += m;
            continue;
        }

        int shimErr = 0;
        shimRc = miniopus__shim_pull(p, miniopus__deliver_pkt, p, &shimErr);
        if (shimRc <= 0) break;
        if (++guard > MO_READ_GUARD) { I->lastResult = MINIOPUS_ERROR; I->streamEnded = MINIOPUS_TRUE; break; }
    }

    if (shimRc < 0) I->streamEnded = MINIOPUS_TRUE;                /* container error */
    else if (shimRc == 0) I->streamEnded = MINIOPUS_TRUE;          /* clean end */

    if (!I->seeking) p->currentPCMFrame += written;
    return written;
}

static miniopus_int16 miniopus__f2s16(float v)
{
    float x = v * 32767.0f;
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (miniopus_int16)MO_lrintf(x);
}
static miniopus_int32 miniopus__f2s32(float v)
{
    float x = v * 2147483647.0f;
    if (x > 2147483647.0f) x = 2147483647.0f;
    if (x < -2147483648.0f) x = -2147483648.0f;
    return (miniopus_int32)MO_lrintf(x);
}

/* ==================================================================== seek
 *
 * Port of UMP-STMH7's opus_seek: bisect the page chain for the page whose
 * granule is the greatest below target_granule, restart the demuxer there
 * (resetForSeek with that page's sequence and serial, which is what keeps the
 * strict RFC validation usable mid-file), reset the decoder, re-arm the
 * pre-skip counter, then decode and discard the frames between the page and the
 * target. Both forward and backward seeks take exactly this path.
 */

MINIOPUS_API miniopus_bool32 miniopus_seek_to_pcm_frame(MiniOpus* pOpus, miniopus_uint64 frameIndex)
{
    if (!pOpus || !pOpus->pInternal) return MINIOPUS_FALSE;
    struct MiniOpus_Internal* I = pOpus->pInternal;
    if (!I->pDec) return MINIOPUS_FALSE;

#ifdef MINIOPUS_NO_SEEKING
    const miniopus_bool32 bisectable = MINIOPUS_FALSE;   /* the bisection is not in this build */
#else
    const miniopus_bool32 bisectable = (I->seekable && I->fileSize != 0) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
#endif
    if (!bisectable) {
        /* No seek, no tell: decode-and-discard forward only. */
        if (frameIndex < pOpus->currentPCMFrame) { I->lastResult = MINIOPUS_INVALID_STATE; return MINIOPUS_FALSE; }
        /* Nothing to jump with, so the seek *is* the decode: frames are read and
         * thrown away until the position passes the target. The emitted count keeps
         * advancing here (suppressing it would leave the loop condition stuck and
         * the walk would run the whole stream away). */
        float discard[120 * 2];
        miniopus_uint32 guard = 0;
        while (pOpus->currentPCMFrame < frameIndex && guard++ < MO_READ_GUARD) {
            miniopus_uint64 want = frameIndex - pOpus->currentPCMFrame;
            if (want > 120) want = 120;
            const miniopus_uint64 got = miniopus__read_frames(pOpus, want, discard);
            if (got == 0) break;
        }
        return pOpus->currentPCMFrame >= frameIndex ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    }

#ifdef MINIOPUS_NO_SEEKING
    { I->lastResult = MINIOPUS_INVALID_STATE; return MINIOPUS_FALSE; }        /* unreachable: the branch above always returns */
#else
    if (pOpus->totalPCMFrameCount == 0) { I->lastResult = MINIOPUS_INVALID_STATE; return MINIOPUS_FALSE; }
    if (frameIndex > pOpus->totalPCMFrameCount) frameIndex = pOpus->totalPCMFrameCount;

    const miniopus_int64 targetGranule = (miniopus_int64)(frameIndex + (miniopus_uint64)I->preSkip);

    miniopus_uint8* scratch = (miniopus_uint8*)miniopus__malloc(&I->alloc, (miniopus_size_t)MINIOPUS_BISECT_SCRATCH);
    if (!scratch) { I->lastResult = MINIOPUS_OUT_OF_MEMORY; return MINIOPUS_FALSE; }

    miniopus_int64 seekGranule = 0;
    miniopus_page_hdr landing;
    miniopus_uint64 pageOff = miniopus__bisect_seek_page(pOpus, targetGranule, &seekGranule, scratch, MINIOPUS_BISECT_SCRATCH, &landing);
#ifdef MINIOPUS_TRACE
    printf("[trace]   bisect landing: page=%llu granule=%lld target=%lld\n",
           (unsigned long long)pageOff, (long long)seekGranule, (long long)targetGranule);
#endif
    if (pageOff == (miniopus_uint64)-1) {
        miniopus__free(&I->alloc, scratch);
        I->lastResult = MINIOPUS_ERROR;
        return MINIOPUS_FALSE;
    }

    miniopus__free(&I->alloc, scratch);

    if (pageOff == 0) {
        /* The stream's own first page: BOS is legitimate there, so the demuxer
         * restarts cold rather than being primed as a mid-stream seek. */
        if (!miniopus__seek_to(pOpus, 0)) { I->lastResult = MINIOPUS_ERROR; return MINIOPUS_FALSE; }
        miniopus__dm_reset(&I->dm);
        I->inOff = 0; I->inLen = 0; I->chunkBase = 0;
    } else if (!miniopus__shim_seek_to(pOpus, pageOff, landing.seq, landing.serial)) {
        I->lastResult = MINIOPUS_ERROR; return MINIOPUS_FALSE;
    }

    opus_decoder_ctl(I->pDec, OPUS_RESET_STATE);
    /* Pre-skip is a property of the head of the stream, not of a landing: the
     * encoder's look-ahead padding only sits in the first packets. Re-arming it
     * here (which UMP-STMH7's opus_seek does) throws away pre_skip extra frames
     * on every seek and hides part of the error below. */
    I->preSkipLeft  = 0;
    I->resid        = 0;
    I->pktFrames    = 0;
    I->streamEnded  = MINIOPUS_FALSE;
    I->eosSeen      = MINIOPUS_FALSE;
    I->lastResult   = MINIOPUS_OK;
    I->lastGranule  = MO_GRANULE_NONE;   /* the walk below re-learns it from the landing page */

    float discard[120 * 2];

    /* Phase 1: walk off the landing page. The bisection guarantees its granule is
     * below the target, and a page's granule is the end time of the LAST packet it
     * completes - so nothing decoded from the landing page reaches the landing
     * granule until that page's own packets are spent. Counting the discard from
     * target - landingGranule without walking the page first (which is what
     * UMP-STMH7's opus_seek does) lands a whole page span early: a second on a
     * 4 KB-page file, ten seconds or more on a muxer that runs 255-segment pages.
     * micro-ogg reports the page's granule on its last completing packet, which is
     * exactly the boundary this loop stops at. */
    I->seeking = MINIOPUS_TRUE;
    if (seekGranule >= 0) {
        /* Packet by packet, not frame by frame: a 120-frame budget would spill
         * over the page boundary the moment a packet crosses it, throwing away
         * frames that belong past the target. */
        int walkErr = 0;
        const int walked = miniopus__shim_walk_to_granule(pOpus, seekGranule, &walkErr);
        if (walked < 0) { I->seeking = MINIOPUS_FALSE; return MINIOPUS_FALSE; }
    }
    /* Whatever granule the walk actually reached is the one the discard counts from. */
    if (I->lastGranule > seekGranule) seekGranule = I->lastGranule;
#ifdef MINIOPUS_TRACE
    printf("[trace] seek(%llu): pageOff=%llu seekGranule=%lld target=%lld phase2Discard=%lld\n",
           (unsigned long long)frameIndex, (unsigned long long)pageOff, (long long)seekGranule,
           (long long)targetGranule, (long long)(targetGranule - seekGranule));
#endif

    /* Phase 2: UMP's discard, now measured from where the decoder actually is. */
    miniopus_int64 toDiscard = targetGranule - ((I->lastGranule > seekGranule) ? I->lastGranule : seekGranule);
    if (toDiscard > 0) {
        miniopus_uint32 guard = 0;
        while (toDiscard > 0 && guard++ < 100000) {
            miniopus_uint64 batch = (miniopus_uint64)((toDiscard > 120) ? 120 : toDiscard);
            const miniopus_uint64 got = miniopus__read_frames(pOpus, batch, discard);
            if (got == 0) break;                                   /* stream ended short of the target */
            toDiscard -= (miniopus_int64)got;
        }
        if (I->lastResult != MINIOPUS_OK && toDiscard > 0) { I->seeking = MINIOPUS_FALSE; return MINIOPUS_FALSE; }
    }
    I->seeking = MINIOPUS_FALSE;

    pOpus->currentPCMFrame = frameIndex;
    return MINIOPUS_TRUE;
#endif /* !MINIOPUS_NO_SEEKING */
}

/* ================================================================== public */

MINIOPUS_API miniopus_uint64 miniopus_read_pcm_frames_f32(MiniOpus* pOpus, miniopus_uint64 framesToRead, float* pFramesOut)
{
    if (!pOpus || !pFramesOut) return 0;
    return miniopus__read_frames(pOpus, framesToRead, pFramesOut);
}

MINIOPUS_API miniopus_uint64 miniopus_read_pcm_frames_s16(MiniOpus* pOpus, miniopus_uint64 framesToRead, miniopus_int16* pFramesOut)
{
    if (!pOpus || !pFramesOut) return 0;
    miniopus_uint64 written = 0;
    while (written < framesToRead) {
        float block[256];
        miniopus_uint64 want = framesToRead - written;
        if (want > 128) want = 128;
        const miniopus_uint64 got = miniopus__read_frames(pOpus, want, block);
        for (miniopus_uint64 i = 0; i < got * pOpus->channels; i++)
            pFramesOut[written * pOpus->channels + i] = miniopus__f2s16(block[i]);
        written += got;
        if (got < want) break;
    }
    return written;
}

MINIOPUS_API miniopus_uint64 miniopus_read_pcm_frames_s32(MiniOpus* pOpus, miniopus_uint64 framesToRead, miniopus_int32* pFramesOut)
{
    if (!pOpus || !pFramesOut) return 0;
    miniopus_uint64 written = 0;
    while (written < framesToRead) {
        float block[256];
        miniopus_uint64 want = framesToRead - written;
        if (want > 128) want = 128;
        const miniopus_uint64 got = miniopus__read_frames(pOpus, want, block);
        for (miniopus_uint64 i = 0; i < got * pOpus->channels; i++)
            pFramesOut[written * pOpus->channels + i] = miniopus__f2s32(block[i]);
        written += got;
        if (got < want) break;
    }
    return written;
}

MINIOPUS_API miniopus_result miniopus_last_error(const MiniOpus* pOpus)
{
    return (pOpus && pOpus->pInternal) ? pOpus->pInternal->lastResult : MINIOPUS_INVALID_ARGUMENT;
}

MINIOPUS_API int miniopus_last_ogg_error(const MiniOpus* pOpus)
{
    return (pOpus && pOpus->pInternal) ? pOpus->pInternal->lastErrorCode : 0;
}

MINIOPUS_API void miniopus_close(MiniOpus* pOpus)
{
    if (!pOpus) return;
    struct MiniOpus_Internal* I = pOpus->pInternal;
    if (!I) return;

    if (I->pDec) opus_decoder_destroy(I->pDec);            /* libopus frees its own state */
    if (I->pScratch) miniopus__free(&I->alloc, I->pScratch);
    if (I->dm.pBuf)  miniopus__free(&I->alloc, I->dm.pBuf);
#ifndef MINIOPUS_NO_METADATA
    miniopus__vs_free(&I->vs, &I->alloc);
#endif
#ifndef MINIOPUS_NO_FILEIO
    if (I->pFile) fclose((FILE*)I->pFile);
#endif
    miniopus__free(&I->alloc, pOpus);
    miniopus__free(&I->alloc, I);
}

MINIOPUS_API miniopus_bool32 miniopus_get_channels           (const MiniOpus* p, miniopus_uint32* v) { if (!p || !v) return MINIOPUS_FALSE; *v = p->channels; return MINIOPUS_TRUE; }
MINIOPUS_API miniopus_bool32 miniopus_get_sample_rate        (const MiniOpus* p, miniopus_uint32* v) { if (!p || !v) return MINIOPUS_FALSE; *v = p->sampleRate; return MINIOPUS_TRUE; }
MINIOPUS_API miniopus_bool32 miniopus_get_source_sample_rate (const MiniOpus* p, miniopus_uint32* v) { if (!p || !v) return MINIOPUS_FALSE; *v = p->sourceSampleRate; return MINIOPUS_TRUE; }
MINIOPUS_API miniopus_bool32 miniopus_get_total_pcm_frames   (const MiniOpus* p, miniopus_uint64* v) { if (!p || !v) return MINIOPUS_FALSE; *v = p->totalPCMFrameCount; return MINIOPUS_TRUE; }
MINIOPUS_API miniopus_bool32 miniopus_get_current_pcm_frame  (const MiniOpus* p, miniopus_uint64* v) { if (!p || !v) return MINIOPUS_FALSE; *v = p->currentPCMFrame; return MINIOPUS_TRUE; }
MINIOPUS_API miniopus_bool32 miniopus_is_seekable            (const MiniOpus* p) { return (p && p->pInternal && p->pInternal->seekable && p->pInternal->fileSize > 0) ? MINIOPUS_TRUE : MINIOPUS_FALSE; }
#ifndef MINIOPUS_NO_METADATA
MINIOPUS_API miniopus_bool32 miniopus_has_picture            (const MiniOpus* p) { return (p && p->pInternal && p->pInternal->pictureFound) ? MINIOPUS_TRUE : MINIOPUS_FALSE; }
#endif

/* --------------------------------------------------------------- open core */

/* The cheap header path (C27): walk the page chain with header-only reads, map
 * where OpusHead and OpusTags sit, read the head, read the head of the tags and
 * hop the album art, then restart the demuxer on the page the audio starts on. */
static miniopus_bool32 miniopus__open_headers_mapped(struct MiniOpus* p)
{
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus_uint8 hdr[MO_PAGE_HDR_MAX];
    miniopus_uint8 head[64];

    if (!miniopus__headmap_map(p, hdr)) return MINIOPUS_FALSE;
    if (I->map.headLen < 19 || I->map.headLen > sizeof(head)) return MINIOPUS_FALSE;
    if (miniopus__read_at(p, I->map.headOff, head, I->map.headLen) != I->map.headLen) return MINIOPUS_FALSE;
    if (memcmp(head, "OpusHead", 8) != 0) return MINIOPUS_FALSE;
    if (!miniopus__parse_head(p, head, I->map.headLen)) return MINIOPUS_FALSE;

#ifndef MINIOPUS_NO_METADATA
    miniopus__vs_reset(&I->vs, NULL, NULL, 0);
    if (!miniopus__vs_ensure_buf(&I->vs, &I->alloc)) return MINIOPUS_FALSE;
    if (!miniopus__headmap_scan_tags(p, hdr, &I->vs)) return MINIOPUS_FALSE;
    miniopus__deliver_meta(p, &I->vs);
    miniopus__vs_free_keep(&I->vs, &I->alloc);          /* the store is consumed; keep the scratch head */
#else
    if (!miniopus__headmap_scan_tags(p, hdr)) return MINIOPUS_FALSE;
#endif

    if (!miniopus__shim_seek_to(p, I->map.audioOff, I->map.audioSeq, I->map.audioSerial)) return MINIOPUS_FALSE;

    I->headersSeen = MINIOPUS_TRUE;
    I->tagsSeen    = MINIOPUS_TRUE;
    I->mapped      = MINIOPUS_TRUE;
    I->seqPos      = I->map.audioOff;
    I->cursorStale = MINIOPUS_TRUE;
#ifdef MINIOPUS_TRACE
    printf("[trace] headers mapped: head @ %llu (%u B), tags @ %llu, audio @ %llu (seq %u serial %08x)\n",
           (unsigned long long)I->map.headOff, I->map.headLen, (unsigned long long)I->map.tagsOff,
           (unsigned long long)I->map.audioOff, I->map.audioSeq, I->map.audioSerial);
#endif
    return MINIOPUS_TRUE;
}

/* Streaming header body callback (the fallback path): OpusHead into hbuf, then
 * OpusTags into the incremental comment parser - a 2 MB cover is parsed as
 * slices and never buffered. */
typedef struct { struct MiniOpus* p; miniopus_uint32 hstate; } miniopus__head_feed;

static int miniopus__head_body(void* pUser, const miniopus_uint8* data, miniopus_size_t n, miniopus_bool32 endOfPacket, miniopus_int64 absOff)
{
    miniopus__head_feed* f = (miniopus__head_feed*)pUser;
    struct MiniOpus* p = f->p;
    struct MiniOpus_Internal* I = p->pInternal;
    (void)absOff;

    if (f->hstate == 0) {
        miniopus_size_t cap = sizeof(I->hbuf) - I->hlen;
        if (n > cap) n = cap;
        memcpy(I->hbuf + I->hlen, data, n);
        I->hlen += (miniopus_uint32)n;
        if (endOfPacket) {
            miniopus__parse_head(p, I->hbuf, I->hlen);
            if (p->channels >= 1 && p->channels <= 2) I->headersSeen = MINIOPUS_TRUE;
            f->hstate = 1;
        }
        return 0;
    }

    if (endOfPacket) {
#ifndef MINIOPUS_NO_METADATA
        miniopus__deliver_meta(p, &I->vs);
#endif
        I->tagsSeen = MINIOPUS_TRUE;
        return 1;                                   /* the boundary, not the parser, is what the demuxer must reach */
    }
#ifndef MINIOPUS_NO_METADATA
    miniopus__vs_feed(&I->vs, data, n, &I->alloc);
#endif
    return 0;
}

static miniopus_bool32 miniopus__open_headers_streamed(struct MiniOpus* p)
{
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus__head_feed feed = { p, 0 };

    I->headersSeen = MINIOPUS_FALSE;
    I->tagsSeen = MINIOPUS_FALSE;
    I->hlen = 0;
#ifndef MINIOPUS_NO_METADATA
    miniopus__vs_reset(&I->vs, NULL, NULL, 0);
    if (!miniopus__vs_ensure_buf(&I->vs, &I->alloc)) return MINIOPUS_FALSE;
#endif

    /* C24: the loop budget has to be one no real file reaches, and running out
     * of it has to fail the open rather than fall through mid-packet - pulling
     * audio in packet mode from a mid-packet streaming state is the illegal mode
     * switch (MOGG_RES_INVALID_MODE_SWITCH), which on hardware reads as a track
     * aborting for no stated reason. */
    int loops = 0;
    const int loopsMax = MINIOPUS_HEADER_SLICE_GUARD;
    int scanErr = 0;
    while ((!I->headersSeen || !I->tagsSeen) && loops < loopsMax) {
        int e = 0;
        const int r = miniopus__shim_pull_raw(p, miniopus__head_body, &feed, &e);
        if (r <= 0) { scanErr = e; break; }
        loops++;
    }

    if (!I->headersSeen) { I->lastResult = MINIOPUS_INVALID_FORMAT; return MINIOPUS_FALSE; }
    if (!I->tagsSeen && scanErr) { I->lastResult = MINIOPUS_INVALID_FORMAT; return MINIOPUS_FALSE; }
    if (!I->tagsSeen) { I->lastResult = MINIOPUS_INVALID_FORMAT; return MINIOPUS_FALSE; }   /* mid-packet: refuse the open */
    return MINIOPUS_TRUE;
}

static MiniOpus* miniopus__open_core(miniopus_read_proc onRead, miniopus_seek_proc onSeek, miniopus_tell_proc onTell,
                                 miniopus_meta_proc onMeta, void* pUserData, void* pMetaUser,
                                 const miniopus_allocation_callbacks* pAllocationCallbacks)
{
    if (!onRead) return NULL;

    miniopus_allocation_callbacks alloc0 = { NULL, NULL, NULL, NULL };
    const miniopus_allocation_callbacks* pAlloc = pAllocationCallbacks ? pAllocationCallbacks : &alloc0;

    MiniOpus* p = (MiniOpus*)miniopus__malloc(pAlloc, sizeof(MiniOpus));
    if (!p) return NULL;
    struct MiniOpus_Internal* I = (struct MiniOpus_Internal*)miniopus__malloc(pAlloc, sizeof(struct MiniOpus_Internal));
    if (!I) { miniopus__free(pAlloc, p); return NULL; }

    memset(p, 0, sizeof(*p));
    memset(I, 0, sizeof(*I));
    p->pInternal = I;
    p->channels = 2;
    p->sampleRate = 48000;
    p->sourceSampleRate = 48000;

    I->onRead = onRead; I->onSeek = onSeek; I->onTell = onTell;
    I->pUserData = pUserData;
    I->pMetaUser = pMetaUser;
    I->onMeta = onMeta;
    I->alloc = *pAlloc;
    I->seekable = (onSeek && onTell) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    I->lastResult = MINIOPUS_OK;

    miniopus__dm_init(&I->dm, pAlloc, MINIOPUS_MIN_PACKET_BUFFER, MINIOPUS_MAX_AUDIO_PACKET,
#ifdef MINIOPUS_CHECK_CRC
                    MINIOPUS_TRUE
#else
                    MINIOPUS_FALSE
#endif
                    );

    I->scratchFrames = MO_MAX_FRAME;
    I->pScratch = (float*)miniopus__malloc(pAlloc, (miniopus_size_t)MO_MAX_FRAME * 2 * sizeof(float));
    if (!I->pScratch) { miniopus_close(p); return NULL; }

    miniopus__find_size(p);

    if (I->seekable && I->fileSize > 0) {
        if (!miniopus__open_headers_mapped(p)) {
            /* The mapped path reads through positional reads and consumes nothing
             * through the demuxer, so the streamed fallback below starts exactly
             * where the file was. */
            miniopus__seek_to(p, 0);
            miniopus__dm_reset(&I->dm);
            I->inOff = 0; I->inLen = 0; I->chunkBase = 0;
            if (!miniopus__open_headers_streamed(p)) { miniopus_close(p); return NULL; }
        }
    } else {
        if (!miniopus__open_headers_streamed(p)) { miniopus_close(p); return NULL; }
    }

    if (p->channels < 1 || p->channels > 2) { I->lastResult = MINIOPUS_UNSUPPORTED; miniopus_close(p); return NULL; }

    int err = 0;
    I->pDec = opus_decoder_create(48000, (int)p->channels, &err);
    if (!I->pDec) {
        I->lastResult = (err == OPUS_ALLOC_FAIL) ? MINIOPUS_OUT_OF_MEMORY : MINIOPUS_INVALID_FORMAT;
        miniopus_close(p);
        return NULL;
    }
    if (I->outputGain != 0) opus_decoder_ctl(I->pDec, OPUS_SET_GAIN((opus_int32)I->outputGain));

    I->preSkipLeft = I->preSkip;
    miniopus__find_total(p);
#ifdef MINIOPUS_TRACE
    printf("[trace] open OK: total=%llu mapped=%d tags=%u oversized=%u\n", (unsigned long long)p->totalPCMFrameCount,
           (int)I->mapped, I->vs.tagsStored, I->vs.oversizedCount);
#endif
    return p;
}

MINIOPUS_API MiniOpus* miniopus_open(miniopus_read_proc onRead, miniopus_seek_proc onSeek, miniopus_tell_proc onTell,
                                void* pUserData, const miniopus_allocation_callbacks* pAllocationCallbacks)
{
    return miniopus__open_core(onRead, onSeek, onTell, NULL, pUserData, pUserData, pAllocationCallbacks);
}

MINIOPUS_API MiniOpus* miniopus_open_with_metadata(miniopus_read_proc onRead, miniopus_seek_proc onSeek, miniopus_tell_proc onTell,
                                              miniopus_meta_proc onMeta, void* pUserData,
                                              const miniopus_allocation_callbacks* pAllocationCallbacks)
{
    return miniopus__open_core(onRead, onSeek, onTell, onMeta, pUserData, pUserData, pAllocationCallbacks);
}
/* ------------------------------------------------------------- memory mode */

static miniopus_size_t miniopus__mem_read(void* pUser, void* pOut, miniopus_size_t n)
{
    MiniOpus* p = (MiniOpus*)pUser;
    struct MiniOpus_Internal* I = p->pInternal;
    if (I->mem.cur >= I->mem.dataSize) return 0;
    miniopus_size_t take = I->mem.dataSize - I->mem.cur < n ? I->mem.dataSize - I->mem.cur : n;
    memcpy(pOut, I->mem.pData + I->mem.cur, take);
    I->mem.cur += take;
    return take;
}
static miniopus_bool32 miniopus__mem_seek(void* pUser, miniopus_int64 off, miniopus_seek_origin origin)
{
    MiniOpus* p = (MiniOpus*)pUser;
    struct MiniOpus_Internal* I = p->pInternal;
    miniopus_int64 target;
    if (origin == MINIOPUS_SEEK_SET) target = off;
    else if (origin == MINIOPUS_SEEK_CUR) target = (miniopus_int64)I->mem.cur + off;
    else target = (miniopus_int64)I->mem.dataSize + off;
    if (target < 0 || (miniopus_size_t)target > I->mem.dataSize) return MINIOPUS_FALSE;
    I->mem.cur = (miniopus_size_t)target;
    return MINIOPUS_TRUE;
}
static miniopus_uint64 miniopus__mem_tell(void* pUser)
{
    MiniOpus* p = (MiniOpus*)pUser;
    return (miniopus_uint64)p->pInternal->mem.cur;
}

MINIOPUS_API MiniOpus* miniopus_open_memory(const void* pData, miniopus_size_t dataSize, const miniopus_allocation_callbacks* pAlloc)
{
    return miniopus_open_memory_with_metadata(pData, dataSize, NULL, NULL, pAlloc);
}
MINIOPUS_API MiniOpus* miniopus_open_memory_with_metadata(const void* pData, miniopus_size_t dataSize, miniopus_meta_proc onMeta, void* pUser, const miniopus_allocation_callbacks* pAlloc)
{
    if (!pData || dataSize == 0) return NULL;
    /* seed the handle early so the callbacks can reach the memory cursor */
    miniopus_allocation_callbacks alloc0 = { NULL, NULL, NULL, NULL };
    const miniopus_allocation_callbacks* pa = pAlloc ? pAlloc : &alloc0;
    MiniOpus* seed = (MiniOpus*)miniopus__malloc(pa, sizeof(MiniOpus));
    if (!seed) return NULL;
    struct MiniOpus_Internal* seedI = (struct MiniOpus_Internal*)miniopus__malloc(pa, sizeof(struct MiniOpus_Internal));
    if (!seedI) { miniopus__free(pa, seed); return NULL; }
    memset(seed, 0, sizeof(*seed)); memset(seedI, 0, sizeof(*seedI));
    seed->pInternal = seedI;
    seedI->mem.pData = (const miniopus_uint8*)pData; seedI->mem.dataSize = dataSize; seedI->mem.cur = 0;
    MiniOpus* p = miniopus__open_core(miniopus__mem_read, miniopus__mem_seek, miniopus__mem_tell, onMeta, seed, pUser, pAlloc);
    /* The seed existed only so the callbacks could reach the cursor before the
    real handle existed. Hand the cursor over and rebind the callbacks onto the
    handle that outlives this function - reading the seed afterwards is a
    use-after-free dressed as a working program. */
    if (p) {
        p->pInternal->mem = seedI->mem;
        p->pInternal->pUserData = p;
        /* Where the feed window says the stream is, not where the storage
         * happens to sit: a positional read (the header map, the tail scan) can
         * leave the cursor anywhere, and the next sequential read has to seek
         * back onto the window's end. */
        p->pInternal->seqPos = (miniopus_uint64)(p->pInternal->chunkBase + (miniopus_int64)p->pInternal->inLen);
        p->pInternal->cursorStale = MINIOPUS_TRUE;
    }
    miniopus__free(pa, seedI);
    miniopus__free(pa, seed);
    return p;
}

/* ---------------------------------------------------------------- file mode */

#ifndef MINIOPUS_NO_FILEIO

static miniopus_size_t miniopus__file_read(void* pUser, void* pOut, miniopus_size_t n)
{
    MiniOpus* p = (MiniOpus*)pUser;
    return fread(pOut, 1, n, (FILE*)p->pInternal->pFile);
}
static miniopus_bool32 miniopus__file_seek(void* pUser, miniopus_int64 off, miniopus_seek_origin origin)
{
    MiniOpus* p = (MiniOpus*)pUser;
    int whence = (origin == MINIOPUS_SEEK_SET) ? SEEK_SET : (origin == MINIOPUS_SEEK_CUR ? SEEK_CUR : SEEK_END);
#if defined(_WIN32)
    return (_fseeki64((FILE*)p->pInternal->pFile, (__int64)off, whence) == 0) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
#elif defined(__unix__) || defined(__APPLE__) || defined(__linux__) || defined(POSIX)
    return (fseeko((FILE*)p->pInternal->pFile, (off_t)off, whence) == 0) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
#else
    return (fseek((FILE*)p->pInternal->pFile, (long)off, whence) == 0) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
#endif
}
static miniopus_uint64 miniopus__file_tell(void* pUser)
{
    MiniOpus* p = (MiniOpus*)pUser;
#if defined(_WIN32)
    long long v = (long long)_ftelli64((FILE*)p->pInternal->pFile);
#elif defined(__unix__) || defined(__APPLE__) || defined(__linux__) || defined(POSIX)
    long long v = (long long)ftello((FILE*)p->pInternal->pFile);
#else
    long v = ftell((FILE*)p->pInternal->pFile);
#endif
    if (v < 0) return (miniopus_uint64)-1;
    return (miniopus_uint64)v;
}

MINIOPUS_API MiniOpus* miniopus_open_file(const char* pPath, const miniopus_allocation_callbacks* pAlloc)
{
    return miniopus_open_file_with_metadata(pPath, NULL, NULL, pAlloc);
}

MINIOPUS_API MiniOpus* miniopus_open_file_with_metadata(const char* pPath, miniopus_meta_proc onMeta, void* pUser, const miniopus_allocation_callbacks* pAlloc)
{
    if (!pPath) return NULL;
    FILE* f = fopen(pPath, "rb");
    if (!f) return NULL;

    miniopus_allocation_callbacks alloc0 = { NULL, NULL, NULL, NULL };
    const miniopus_allocation_callbacks* pa = pAlloc ? pAlloc : &alloc0;
    MiniOpus* seed = (MiniOpus*)miniopus__malloc(pa, sizeof(MiniOpus));
    if (!seed) { fclose(f); return NULL; }
    struct MiniOpus_Internal* seedI = (struct MiniOpus_Internal*)miniopus__malloc(pa, sizeof(struct MiniOpus_Internal));
    if (!seedI) { miniopus__free(pa, seed); fclose(f); return NULL; }
    memset(seed, 0, sizeof(*seed)); memset(seedI, 0, sizeof(*seedI));
    seed->pInternal = seedI;
    seedI->pFile = f;

    MiniOpus* p = miniopus__open_core(miniopus__file_read, miniopus__file_seek, miniopus__file_tell, onMeta, seed, pUser, pAlloc);
    if (p) {
        p->pInternal->pFile = f;                /* the real handle owns the FILE now */
        p->pInternal->pUserData = p;
        p->pInternal->seqPos = (miniopus_uint64)(p->pInternal->chunkBase + (miniopus_int64)p->pInternal->inLen);
        p->pInternal->cursorStale = MINIOPUS_TRUE;
    } else {
        fclose(f);
    }
    miniopus__free(pa, seedI);
    miniopus__free(pa, seed);
    return p;
}

#endif /* MINIOPUS_NO_FILEIO */
/* ------------------------------------------------------------- convenience */

MINIOPUS_API void miniopus_free(void* pPtr, const miniopus_allocation_callbacks* pAlloc)
{
    miniopus_allocation_callbacks alloc0 = { NULL, NULL, NULL, NULL };
    miniopus__free(pAlloc ? pAlloc : &alloc0, pPtr);
}

static miniopus_uint64 miniopus__read_all_into(MiniOpus* p, void** ppOut, const miniopus_allocation_callbacks* pAlloc, int fmt)
{
    miniopus_uint64 total = p->totalPCMFrameCount;
    miniopus_uint64 cap = total ? total : 65536;
    miniopus_size_t bytesPerFrame = (miniopus_size_t)p->channels * (fmt == 0 ? 2 : 4);
    void* buf = NULL;
    miniopus_uint64 got = 0;
    for (;;) {
        buf = miniopus__realloc(pAlloc, buf, (miniopus_size_t)cap * bytesPerFrame);
        if (!buf) { miniopus_close(p); return 0; }
        miniopus_uint64 want = cap - got;
        miniopus_uint64 r;
        if (fmt == 0) r = miniopus_read_pcm_frames_s16(p, want, (miniopus_int16*)buf + got * p->channels);
        else if (fmt == 1) r = miniopus_read_pcm_frames_s32(p, want, (miniopus_int32*)buf + got * p->channels);
        else r = miniopus_read_pcm_frames_f32(p, want, (float*)buf + got * p->channels);
        got += r;
        if (r < want) break;
        cap *= 2;
    }
    *ppOut = buf;
    return got;
}

#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API miniopus_int16* miniopus_open_file_and_read_pcm_frames_s16(const char* pPath, miniopus_uint32* pCh, miniopus_uint32* pRate, miniopus_uint64* pFrames, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpus* p = miniopus_open_file(pPath, pAlloc); if (!p) return NULL;
    void* buf = NULL; miniopus_uint64 n = miniopus__read_all_into(p, &buf, pAlloc, 0);
    if (pCh) *pCh = p->channels;
    if (pRate) *pRate = p->sampleRate;
    if (pFrames) *pFrames = n;
    miniopus_close(p);
    return (miniopus_int16*)buf;
}
MINIOPUS_API miniopus_int32* miniopus_open_file_and_read_pcm_frames_s32(const char* pPath, miniopus_uint32* pCh, miniopus_uint32* pRate, miniopus_uint64* pFrames, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpus* p = miniopus_open_file(pPath, pAlloc); if (!p) return NULL;
    void* buf = NULL; miniopus_uint64 n = miniopus__read_all_into(p, &buf, pAlloc, 1);
    if (pCh) *pCh = p->channels;
    if (pRate) *pRate = p->sampleRate;
    if (pFrames) *pFrames = n;
    miniopus_close(p);
    return (miniopus_int32*)buf;
}
MINIOPUS_API float* miniopus_open_file_and_read_pcm_frames_f32(const char* pPath, miniopus_uint32* pCh, miniopus_uint32* pRate, miniopus_uint64* pFrames, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpus* p = miniopus_open_file(pPath, pAlloc); if (!p) return NULL;
    void* buf = NULL; miniopus_uint64 n = miniopus__read_all_into(p, &buf, pAlloc, 2);
    if (pCh) *pCh = p->channels;
    if (pRate) *pRate = p->sampleRate;
    if (pFrames) *pFrames = n;
    miniopus_close(p);
    return (float*)buf;
}
#endif

MINIOPUS_API miniopus_int16* miniopus_open_memory_and_read_pcm_frames_s16(const void* pData, miniopus_size_t dataSize, miniopus_uint32* pCh, miniopus_uint32* pRate, miniopus_uint64* pFrames, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpus* p = miniopus_open_memory(pData, dataSize, pAlloc); if (!p) return NULL;
    void* buf = NULL; miniopus_uint64 n = miniopus__read_all_into(p, &buf, pAlloc, 0);
    if (pCh) *pCh = p->channels;
    if (pRate) *pRate = p->sampleRate;
    if (pFrames) *pFrames = n;
    miniopus_close(p);
    return (miniopus_int16*)buf;
}
MINIOPUS_API miniopus_int32* miniopus_open_memory_and_read_pcm_frames_s32(const void* pData, miniopus_size_t dataSize, miniopus_uint32* pCh, miniopus_uint32* pRate, miniopus_uint64* pFrames, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpus* p = miniopus_open_memory(pData, dataSize, pAlloc); if (!p) return NULL;
    void* buf = NULL; miniopus_uint64 n = miniopus__read_all_into(p, &buf, pAlloc, 1);
    if (pCh) *pCh = p->channels;
    if (pRate) *pRate = p->sampleRate;
    if (pFrames) *pFrames = n;
    miniopus_close(p);
    return (miniopus_int32*)buf;
}
MINIOPUS_API float* miniopus_open_memory_and_read_pcm_frames_f32(const void* pData, miniopus_size_t dataSize, miniopus_uint32* pCh, miniopus_uint32* pRate, miniopus_uint64* pFrames, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpus* p = miniopus_open_memory(pData, dataSize, pAlloc); if (!p) return NULL;
    void* buf = NULL; miniopus_uint64 n = miniopus__read_all_into(p, &buf, pAlloc, 2);
    if (pCh) *pCh = p->channels;
    if (pRate) *pRate = p->sampleRate;
    if (pFrames) *pFrames = n;
    miniopus_close(p);
    return (float*)buf;
}

/* A stable bitstream serial: derived from the stream's own parameters (and
the path for file output, where the caller's file name is genuinely stream
identity). Two files encoded with identical parameters on the same host get
the same serial, which is harmless in Ogg-land (serials only have to be
distinct *within* a file) and makes encoder output byte-reproducible, which
the tests rely on. */
static miniopus_uint32 miniopus__serial_seed(miniopus_uint32 rate, miniopus_uint32 ch, const void* pSeedStr)
{
    miniopus_uint32 h = miniopus__crc_update(0, (const miniopus_uint8*)"MiniOpus", 6);
    miniopus_uint8 aux[8];
    miniopus__put_le32(aux, rate);
    miniopus__put_le32(aux + 4, ch);
    h = miniopus__crc_update(h, aux, 8);
    if (pSeedStr) h = miniopus__crc_update(h, (const miniopus_uint8*)pSeedStr, strlen((const char*)pSeedStr));
    return h;
}

/* ================================================================= encoder
The other half of the UMP-STMH7 pipeline's lesson set: a conformant Ogg Opus
muxer in a few hundred lines of C. Pages are assembled in a bounded body
buffer, laced in 255-chunks, CRC'd (MPEG-2, as the format demands), and
granule-accounted in 48 kHz samples exactly the way the demuxer reads them:
each audio packet adds its frame duration, the header packets add nothing,
and OpusHead's pre-skip carries the encoder's real look-ahead converted to
48 kHz - so what this writes is seekable by miniopus_seek_to_pcm_frame(),
length-exact under opusfile, and decodes sample-identically to MiniOpus's own
decoder (all asserted in the test suite). */

#ifndef MINIOPUS_NO_ENCODER

#include <stdarg.h>

#define MO_MAX_OPUS_PACKET 1275

struct MiniOpusEncoder_Internal {
    OpusEncoder*      pEnc;
    miniopus_write_proc onWrite;
    void*             pUserData;
    miniopus_allocation_callbacks alloc;
    void*             pFile;                          /* FILE* in file mode */

    miniopus_uint8* pOut; miniopus_size_t outLen, outCap; /* memory mode */

    const miniopus_tag* cfgTags; miniopus_uint32 cfgTagCount; const char* cfgVendor;

    miniopus_uint32 serial, seq;
    miniopus_int64  granuleDone;                        /* completed 48k samples */
    miniopus_bool32 firstPage;
    miniopus_bool32 pageContinues;                      /* last flush ended mid-packet */
    miniopus_bool32 pageCompletedHere;

    miniopus_uint8* pPageBody; miniopus_size_t bodyLen;
    miniopus_uint8  lacing[256]; miniopus_uint32 segCount;

    miniopus_uint32 frameMsX10, frameIn, frameDur48;
    miniopus_uint32 preskip48;

    float*        pStage;
    miniopus_uint32 stageFrames;                        /* per channel */
    miniopus_uint32 stageFill;

    miniopus_uint8* pPkt;
    miniopus_bool32 headersWritten;
    miniopus_bool32 finalized;
    miniopus_bool32 failed;
    miniopus_result lastResult;
};

static miniopus_size_t miniopus__enc_write_out(void* pUser, const void* pData, miniopus_size_t n)
{
    MiniOpusEncoder* e = (MiniOpusEncoder*)pUser;
    struct MiniOpusEncoder_Internal* E = e->pInternal;
#ifndef MINIOPUS_NO_FILEIO
    if (E->pFile) return fwrite(pData, 1, n, (FILE*)E->pFile);
#endif
    /* memory mode: append, growing geometrically */
    if (E->outLen + n > E->outCap) {
        miniopus_size_t cap = E->outCap ? E->outCap : 4096;
        while (cap < E->outLen + n) cap *= 2;
        miniopus_uint8* p = (miniopus_uint8*)miniopus__realloc(&E->alloc, E->pOut, cap);
        if (!p) { E->lastResult = MINIOPUS_OUT_OF_MEMORY; E->failed = MINIOPUS_TRUE; return 0; }
        E->pOut = p; E->outCap = cap;
    }
    memcpy(E->pOut + E->outLen, pData, n);
    E->outLen += n;
    return n;
}

static miniopus_bool32 miniopus__mux_flush_page(MiniOpusEncoder* e, miniopus_bool32 eos)
{
    struct MiniOpusEncoder_Internal* E = e->pInternal;
    miniopus_uint8 hdr[MO_PAGE_HDR_MAX];
    memset(hdr, 0, 27);
    memcpy(hdr, "OggS", 4);
    hdr[4] = 0;
    miniopus_uint8 flags = 0;
    if (E->firstPage) flags |= 0x02;
    if (E->pageContinues) flags |= 0x01;
    if (eos) flags |= 0x04;
    hdr[5] = flags;
    miniopus__put_le64(hdr + 6, E->pageCompletedHere ? E->granuleDone : MO_GRANULE_NONE);
    miniopus__put_le32(hdr + 14, E->serial);
    miniopus__put_le32(hdr + 18, E->seq++);
    /* crc field (22..25) stays zero while computing */
    hdr[26] = (miniopus_uint8)E->segCount;
    memcpy(hdr + 27, E->lacing, E->segCount);

    miniopus_uint32 crc = miniopus__crc_update(0, hdr, (miniopus_size_t)(27 + E->segCount));
    crc = miniopus__crc_update(crc, E->pPageBody, E->bodyLen);
    miniopus__put_le32(hdr + 22, crc);

    if (E->onWrite(E->pUserData, hdr, (miniopus_size_t)(27 + E->segCount)) != (miniopus_size_t)(27 + E->segCount)) { E->failed = MINIOPUS_TRUE; E->lastResult = MINIOPUS_ERROR; return MINIOPUS_FALSE; }
    if (E->bodyLen > 0 && E->onWrite(E->pUserData, E->pPageBody, E->bodyLen) != E->bodyLen) { E->failed = MINIOPUS_TRUE; E->lastResult = MINIOPUS_ERROR; return MINIOPUS_FALSE; }

    E->pageContinues = (E->segCount > 0 && E->lacing[E->segCount - 1] == 255) ? MINIOPUS_TRUE : MINIOPUS_FALSE;
    E->segCount = 0; E->bodyLen = 0; E->pageCompletedHere = MINIOPUS_FALSE; E->firstPage = MINIOPUS_FALSE;
    return MINIOPUS_TRUE;
}

static miniopus_bool32 miniopus__mux_append_packet(MiniOpusEncoder* e, const miniopus_uint8* pkt, miniopus_size_t len, miniopus_int64 dur48)
{
    struct MiniOpusEncoder_Internal* E = e->pInternal;
    miniopus_uint8 lastSeg = 0;
    while (len > 0) {
        /* segments are 255 bytes except the one that ENDS the packet (any
        length below 255). A page that is full must therefore be flushed
        BEFORE the take is clamped - clamping a segment to the remaining room
        would fake a packet end mid-packet and shred the lacing semantics of
        everything after it (this is the bug that ate MiniOpus' first oversized
        tags packets). */
        miniopus_size_t take = len < 255 ? len : 255;
        miniopus_size_t room = (miniopus_size_t)MINIOPUS_PAGE_BODY_MAX - E->bodyLen;
        if (E->segCount >= 255 || (miniopus_size_t)take > room) {
            if (!miniopus__mux_flush_page(e, MINIOPUS_FALSE)) return MINIOPUS_FALSE;
            room = (miniopus_size_t)MINIOPUS_PAGE_BODY_MAX;
        }
        memcpy(E->pPageBody + E->bodyLen, pkt, take);
        E->bodyLen += take;
        E->lacing[E->segCount++] = (miniopus_uint8)take;
        lastSeg = (miniopus_uint8)take;
        pkt += take; len -= take;
        if (take < 255) { E->granuleDone += dur48; E->pageCompletedHere = MINIOPUS_TRUE; }
    }
    /* A packet whose byte count is a multiple of 255 ends its last data segment
    at exactly 255 - lacing-wise that *looks like a continuation* and the next
    packet fuses onto it. The convention is a terminating zero-length segment
    in that case (libogg does the same). Missing it inflates granule totals and
    makes opusfile reject the stream. */
    if (lastSeg == 255) {
        if (E->segCount >= 255 && !miniopus__mux_flush_page(e, MINIOPUS_FALSE)) return MINIOPUS_FALSE;
        E->lacing[E->segCount++] = 0;
        E->granuleDone += dur48;
        E->pageCompletedHere = MINIOPUS_TRUE;
    }
    return MINIOPUS_TRUE;
}

static miniopus_bool32 miniopus__enc_write_headers(MiniOpusEncoder* e)
{
    struct MiniOpusEncoder_Internal* E = e->pInternal;
    const miniopus_uint32 rate = e->sampleRate;
    const miniopus_uint32 ch = e->channels;

    miniopus_uint8 head[19];
    memcpy(head, "OpusHead", 8);
    head[8] = 0; head[9] = (miniopus_uint8)ch;
    miniopus__put_le16(head + 10, (miniopus_uint16)E->preskip48);   /* pre-skip is 16 bits in OpusHead */
    miniopus__put_le32(head + 12, rate);
    miniopus__put_le16(head + 16, 0);          /* output gain */
    head[18] = 0;                            /* channel mapping family 0 */
    if (!miniopus__mux_append_packet(e, head, sizeof(head), 0)) return MINIOPUS_FALSE;
    if (!miniopus__mux_flush_page(e, MINIOPUS_FALSE)) return MINIOPUS_FALSE;

    /* OpusTags: magic + framing + vendor + comments. Tags arrive as caller
    "KEY=VALUE" strings, written verbatim. */
    const miniopus_tag* pTags = NULL; miniopus_uint32 tagCount = 0; const char* pVendor = "MiniOpus";
    if (e->pInternal->cfgTags) { pTags = e->pInternal->cfgTags; tagCount = e->pInternal->cfgTagCount; }
    if (e->pInternal->cfgVendor && e->pInternal->cfgVendor[0]) pVendor = e->pInternal->cfgVendor;

    miniopus_size_t vlen = strlen(pVendor);
    miniopus_size_t cap = 8 + 4 + 4 + vlen + 4;
    for (miniopus_uint32 i = 0; i < tagCount; i++) cap += 4 + strlen(pTags[i].key) + 1 + strlen(pTags[i].value);
    miniopus_uint8* tags = (miniopus_uint8*)miniopus__malloc(&E->alloc, cap);
    if (!tags) { E->lastResult = MINIOPUS_OUT_OF_MEMORY; return MINIOPUS_FALSE; }
    miniopus_size_t o = 0;
    memcpy(tags + o, "OpusTags", 8); o += 8;
    /* RFC-correct layout: magic, then vendor_length directly. Encoders in the
    wild (opusenc, lavf) write no framing field at the head of the tags packet
    and lavf writes none at the tail either; count-based parsing survives both
    (verified against a 1.9 MB-tags file that carries 4 KB of slack after the
    last comment). */
    miniopus__put_le32(tags + o, (miniopus_uint32)vlen); o += 4;
    memcpy(tags + o, pVendor, vlen); o += vlen;
    miniopus__put_le32(tags + o, tagCount); o += 4;
    for (miniopus_uint32 i = 0; i < tagCount; i++) {
        const char* k = pTags[i].key ? pTags[i].key : "";
        const char* v = pTags[i].value ? pTags[i].value : "";
        miniopus_size_t kl = strlen(k), vl = strlen(v);
        miniopus_size_t cl = kl + 1 + vl;
        miniopus__put_le32(tags + o, (miniopus_uint32)cl); o += 4;
        memcpy(tags + o, k, kl); o += kl;
        tags[o++] = '=';
        memcpy(tags + o, v, vl); o += vl;
    }
    miniopus_bool32 ok = miniopus__mux_append_packet(e, tags, o, 0) && miniopus__mux_flush_page(e, MINIOPUS_FALSE);
    miniopus__free(&E->alloc, tags);
    return ok;
}

static miniopus_bool32 miniopus__enc_encode_frame(MiniOpusEncoder* e)
{
    struct MiniOpusEncoder_Internal* E = e->pInternal;
    int n = opus_encode_float(E->pEnc, E->pStage, (int)E->frameIn, E->pPkt, MO_MAX_OPUS_PACKET);
    if (n < 0) { E->failed = MINIOPUS_TRUE; E->lastResult = MINIOPUS_ERROR; return MINIOPUS_FALSE; }
    if (!miniopus__mux_append_packet(e, E->pPkt, (miniopus_size_t)n, (miniopus_int64)E->frameDur48)) return MINIOPUS_FALSE;
    return MINIOPUS_TRUE;
}

/* Consume interleaved floats into the staging block and emit whole frames. */
static miniopus_bool32 miniopus__enc_write_f32_range(MiniOpusEncoder* e, const float* pInterleaved, miniopus_uint64 samples)
{
    struct MiniOpusEncoder_Internal* E = e->pInternal;
    if (E->finalized || E->failed) { E->lastResult = MINIOPUS_INVALID_STATE; return MINIOPUS_FALSE; }
    if (!E->headersWritten) { if (!miniopus__enc_write_headers(e)) { E->failed = MINIOPUS_TRUE; return MINIOPUS_FALSE; } E->headersWritten = MINIOPUS_TRUE; }
    const miniopus_uint32 ch = e->channels;
    miniopus_uint64 done = 0;
    while (done < samples) {
        miniopus_uint32 need = E->stageFrames - E->stageFill;               /* per channel */
        miniopus_uint64 needSamples = (miniopus_uint64)need * ch;
        miniopus_uint64 avail = samples - done;
        miniopus_uint64 take = avail < needSamples ? avail : needSamples;
        memcpy(E->pStage + (miniopus_size_t)E->stageFill * ch, pInterleaved + done, (miniopus_size_t)take * sizeof(float));
        done += take;
        E->stageFill += (miniopus_uint32)(ch ? take / ch : 0);
        if (E->stageFill == E->stageFrames) {
            if (!miniopus__enc_encode_frame(e)) return MINIOPUS_FALSE;
            E->stageFill = 0;
        }
    }
    return MINIOPUS_TRUE;
}

/* public encoder create core */
static MiniOpusEncoder* miniopus__enc_create(miniopus_write_proc onWrite, void* pUserData,
                                         miniopus_uint32 sampleRate, miniopus_uint32 channels,
                                         const miniopus_encoder_config* pConfig,
                                         const miniopus_allocation_callbacks* pAllocationCallbacks,
                                         void* pFile)
{
    if (sampleRate != 8000 && sampleRate != 12000 && sampleRate != 16000 && sampleRate != 24000 && sampleRate != 48000) return NULL;
    if (channels < 1 || channels > 2) return NULL;

    miniopus_allocation_callbacks alloc0 = { NULL, NULL, NULL, NULL };
    const miniopus_allocation_callbacks* pAlloc = pAllocationCallbacks ? pAllocationCallbacks : &alloc0;

    MiniOpusEncoder* e = (MiniOpusEncoder*)miniopus__malloc(pAlloc, sizeof(MiniOpusEncoder));
    if (!e) return NULL;
    struct MiniOpusEncoder_Internal* E = (struct MiniOpusEncoder_Internal*)miniopus__malloc(pAlloc, sizeof(struct MiniOpusEncoder_Internal));
    if (!E) { miniopus__free(pAlloc, e); return NULL; }
    memset(e, 0, sizeof(*e)); memset(E, 0, sizeof(*E));
    e->pInternal = E;
    e->sampleRate = sampleRate; e->channels = channels;

    E->alloc = *pAlloc;
    E->pFile = pFile;
    E->onWrite = onWrite ? onWrite : miniopus__enc_write_out;
    E->pUserData = onWrite ? pUserData : e;
    E->firstPage = MINIOPUS_TRUE;
    E->granuleDone = 0;
    E->seq = 0;

    miniopus_encoder_config def = { 0, -1, -1, 0, MINIOPUS_APPLICATION_AUDIO, NULL, 0, NULL };
    const miniopus_encoder_config* cfg = pConfig ? pConfig : &def;
    miniopus_int32 frameX10 = cfg->frameMsX10 ? cfg->frameMsX10 : 200;
    if (frameX10 != 25 && frameX10 != 50 && frameX10 != 100 && frameX10 != 200 && frameX10 != 400 && frameX10 != 600) {
        miniopus__free(pAlloc, E); miniopus__free(pAlloc, e); return NULL;
    }
    e->frameMsX10 = (miniopus_uint32)frameX10;
    E->frameMsX10 = (miniopus_uint32)frameX10;
    E->frameIn    = (miniopus_uint32)(((miniopus_uint64)sampleRate * (miniopus_uint64)frameX10) / 10000);
    E->frameDur48 = (miniopus_uint32)(((miniopus_uint64)48000 * (miniopus_uint64)frameX10) / 10000);

    E->cfgTags = cfg->pTags; E->cfgTagCount = cfg->tagCount; E->cfgVendor = cfg->pVendorString;

    int app = (int)cfg->application;
    if (app != 2048 && app != 2049 && app != 2051) app = 2049;   /* default AUDIO */
    int err = 0;
    E->pEnc = opus_encoder_create((opus_int32)sampleRate, (int)channels, app, &err);
    if (!E->pEnc) { miniopus__free(pAlloc, E); miniopus__free(pAlloc, e); return NULL; }
    if (cfg->bitrateBps > 0) opus_encoder_ctl(E->pEnc, OPUS_SET_BITRATE(cfg->bitrateBps));
    if (cfg->vbr >= 0) opus_encoder_ctl(E->pEnc, OPUS_SET_VBR(cfg->vbr ? 1 : 0));
    if (cfg->complexity >= 0) opus_encoder_ctl(E->pEnc, OPUS_SET_COMPLEXITY(cfg->complexity > 10 ? 10 : cfg->complexity));

    int lookahead = 0;
    opus_encoder_ctl(E->pEnc, OPUS_GET_LOOKAHEAD(&lookahead));
    E->preskip48 = (miniopus_uint32)(((miniopus_uint64)(lookahead < 0 ? 0 : lookahead) * 48000) / sampleRate);

    E->serial = miniopus__serial_seed(sampleRate, channels, pFile ? (const void*)pUserData : NULL);

    E->pPageBody = (miniopus_uint8*)miniopus__malloc(pAlloc, (miniopus_size_t)MINIOPUS_PAGE_BODY_MAX);
    E->pStage    = (float*)miniopus__malloc(pAlloc, (miniopus_size_t)E->frameIn * channels * sizeof(float));
    E->pPkt      = (miniopus_uint8*)miniopus__malloc(pAlloc, MO_MAX_OPUS_PACKET);
    E->stageFrames = E->frameIn;
    if (!E->pPageBody || !E->pStage || !E->pPkt) { miniopus_encoder_close(e); return NULL; }
    return e;
}

MINIOPUS_API MiniOpusEncoder* miniopus_encoder_create(miniopus_write_proc onWrite, void* pUserData, miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAlloc)
{
    return miniopus__enc_create(onWrite, pUserData, sampleRate, channels, pConfig, pAlloc, NULL);
}

MINIOPUS_API MiniOpusEncoder* miniopus_encoder_create_memory(miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAlloc)
{
    return miniopus__enc_create(NULL, NULL, sampleRate, channels, pConfig, pAlloc, NULL);
}

MINIOPUS_API const miniopus_uint8* miniopus_encoder_get_memory(const MiniOpusEncoder* pEncoder, miniopus_size_t* pSizeOut)
{
    if (!pEncoder || !pEncoder->pInternal) { if (pSizeOut) *pSizeOut = 0; return NULL; }
    if (pSizeOut) *pSizeOut = pEncoder->pInternal->outLen;
    return pEncoder->pInternal->pOut;
}

#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API MiniOpusEncoder* miniopus_encoder_create_file(const char* pPath, miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAlloc)
{
    if (!pPath) return NULL;
    FILE* f = fopen(pPath, "wb");
    if (!f) return NULL;
    MiniOpusEncoder* e = miniopus__enc_create(NULL, (void*)pPath, sampleRate, channels, pConfig, pAlloc, f);
    if (!e) fclose(f);
    return e;
}
#endif

MINIOPUS_API miniopus_bool32 miniopus_encoder_write_f32(MiniOpusEncoder* pEncoder, const float* pInterleaved, miniopus_uint64 frames)
{
    if (!pEncoder || !pEncoder->pInternal || !pInterleaved) return MINIOPUS_FALSE;
    return miniopus__enc_write_f32_range(pEncoder, pInterleaved, frames * pEncoder->channels);
}

MINIOPUS_API miniopus_bool32 miniopus_encoder_write_s16(MiniOpusEncoder* pEncoder, const miniopus_int16* pInterleaved, miniopus_uint64 frames)
{
    if (!pEncoder || !pEncoder->pInternal || !pInterleaved) return MINIOPUS_FALSE;
    miniopus_uint64 total = frames * pEncoder->channels;
    miniopus_uint32 ch = pEncoder->channels;
    miniopus_uint64 done = 0;
    while (done < total) {
        float block[512];
        miniopus_uint64 take = total - done; if (take > 256) take = 256;
        for (miniopus_uint64 i = 0; i < take; i++) block[i] = (float)pInterleaved[(done + i)] / 32768.0f;
        if (!miniopus__enc_write_f32_range(pEncoder, block, take)) return MINIOPUS_FALSE;
        done += take;
    }
    (void)ch;
    return MINIOPUS_TRUE;
}

MINIOPUS_API miniopus_bool32 miniopus_encoder_write_s32(MiniOpusEncoder* pEncoder, const miniopus_int32* pInterleaved, miniopus_uint64 frames)
{
    if (!pEncoder || !pEncoder->pInternal || !pInterleaved) return MINIOPUS_FALSE;
    miniopus_uint64 total = frames * pEncoder->channels;
    miniopus_uint64 done = 0;
    while (done < total) {
        float block[512];
        miniopus_uint64 take = total - done; if (take > 256) take = 256;
        for (miniopus_uint64 i = 0; i < take; i++) block[i] = (float)pInterleaved[done + i] / 2147483648.0f;
        if (!miniopus__enc_write_f32_range(pEncoder, block, take)) return MINIOPUS_FALSE;
        done += take;
    }
    return MINIOPUS_TRUE;
}

#ifndef MINIOPUS_NO_FILEIO
MINIOPUS_API miniopus_result miniopus_encode_pcm16_to_opus_file(const char* pOutputPath, const miniopus_int16* pInterleaved, miniopus_uint64 frames, miniopus_uint32 sampleRate, miniopus_uint32 channels, const miniopus_encoder_config* pConfig, const miniopus_allocation_callbacks* pAlloc)
{
    MiniOpusEncoder* e = miniopus_encoder_create_file(pOutputPath, sampleRate, channels, pConfig, pAlloc);
    if (!e) return MINIOPUS_INVALID_ARGUMENT;
    miniopus_bool32 ok = miniopus_encoder_write_s16(e, pInterleaved, frames);
    miniopus_result r = ok ? MINIOPUS_OK : (e->pInternal->lastResult ? e->pInternal->lastResult : MINIOPUS_ERROR);
    miniopus_encoder_close(e);
    return r;
}
#endif /* !MINIOPUS_NO_FILEIO */

MINIOPUS_API miniopus_result miniopus_encoder_finalize(MiniOpusEncoder* pEncoder)
{
    if (!pEncoder || !pEncoder->pInternal) return MINIOPUS_INVALID_ARGUMENT;
    struct MiniOpusEncoder_Internal* E = pEncoder->pInternal;
    if (E->finalized) return E->failed ? E->lastResult : MINIOPUS_OK;
    E->finalized = MINIOPUS_TRUE;

    if (!E->failed) {
        if (!E->headersWritten) { if (miniopus__enc_write_headers(pEncoder)) E->headersWritten = MINIOPUS_TRUE; }
        /* pad the tail with silence into the last frame - the standard
        encoder practice (opusenc does the same); the pad shows up as a
        frame-sized tail of (near)silence in the decoded stream */
        if (E->stageFill > 0) {
            memset(E->pStage + (miniopus_size_t)E->stageFill * pEncoder->channels, 0,
                   (miniopus_size_t)(E->stageFrames - E->stageFill) * pEncoder->channels * sizeof(float));
            miniopus__enc_encode_frame(pEncoder);
        }
        miniopus__mux_flush_page(pEncoder, MINIOPUS_TRUE);     /* final page, EOS */
    }
    return E->failed ? (E->lastResult ? E->lastResult : MINIOPUS_ERROR) : MINIOPUS_OK;
}

MINIOPUS_API void miniopus_encoder_close(MiniOpusEncoder* pEncoder)
{
    if (!pEncoder || !pEncoder->pInternal) return;
    struct MiniOpusEncoder_Internal* E = pEncoder->pInternal;
    miniopus_allocation_callbacks alloc = E->alloc;

    miniopus_encoder_finalize(pEncoder);

#ifndef MINIOPUS_NO_FILEIO
#ifndef MINIOPUS_NO_FILEIO
    if (E->pFile) fclose((FILE*)E->pFile);
#endif
#endif
    if (E->pEnc) opus_encoder_destroy(E->pEnc);
    miniopus__free(&alloc, E->pPageBody);
    miniopus__free(&alloc, E->pStage);
    miniopus__free(&alloc, E->pPkt);
    miniopus__free(&alloc, E->pOut);
    miniopus__free(&alloc, E);
    miniopus__free(&alloc, pEncoder);
}

#endif /* MINIOPUS_NO_ENCODER */

#endif /* MINIOPUS_IMPLEMENTATION */
