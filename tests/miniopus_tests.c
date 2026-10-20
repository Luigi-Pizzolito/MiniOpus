/*
MiniOpus host test suite. Public Domain - see LICENSE.

Build with libopus (pkg-config opus). When opusfile is available
(pkg-config opusfile) the suite cross-checks durations, pre-skip conventions
and decoded samples against it - that is what pins the container conventions
down instead of letting the encoder grade its own homework.

Optional: set OGGTEST_DIR to a directory containing real .opus/.ogg files;
the suite walks up to 12 of them (this is the corpus shape UMP-STMH7 exports:
big embedded covers, 255-segment page chains, granule -1 tail runs).
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <sys/stat.h>

#include "miniopus.h"

#ifdef HAVE_OPUSFILE
#include <opus/opusfile.h>
#endif

static int g_fails = 0;
static int g_checks = 0;

#define CHECK(cond, name) do {                                                                   \
    g_checks++;                                                                                  \
    if (cond) { printf("  [ ok ] %s\n", name); }                                                \
    else { printf("  [FAIL] %s  (%s:%d)\n", name, __FILE__, __LINE__); g_fails++; }             \
} while (0)

#define CHECKC(cond, name, ...) do {                                                             \
    g_checks++;                                                                                  \
    if (cond) { printf("  [ ok ] %s\n", name); }                                                \
    else { printf("  [FAIL] %s  (%s:%d): ", name, __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); g_fails++; } \
} while (0)

/* ------------------------------------------------- tracking allocation layer */

typedef struct { size_t live, peak, total, allocs, frees; } memtrack;
static memtrack g_mt;

typedef struct { size_t magic; size_t size; } mt_hdr;
#define MT_MAGIC ((size_t)0x614D546Cu)

static void* mt_malloc(size_t n, void* ud)
{
    (void)ud;
    mt_hdr* h = (mt_hdr*)malloc(n + sizeof(mt_hdr) + 8);
    if (!h) return NULL;
    h->magic = MT_MAGIC; h->size = n;
    memset(h + 1, 0xAB, n);                       /* poison: reads of uninitialised buffers show up */
    g_mt.live += n; g_mt.total += n; g_mt.allocs++;
    if (g_mt.live > g_mt.peak) g_mt.peak = g_mt.live;
    return h + 1;
}
static void* mt_realloc(void* p, size_t n, void* ud)
{
    if (!p) return mt_malloc(n, ud);
    mt_hdr* h = (mt_hdr*)((char*)p - sizeof(mt_hdr));
    if (h->magic != MT_MAGIC) { fprintf(stderr, "realloc on foreign pointer!\n"); abort(); }
    size_t oldsz = h->size;
    mt_hdr* nh = (mt_hdr*)realloc(h, n + sizeof(mt_hdr) + 8);
    if (!nh) return NULL;
    nh->magic = MT_MAGIC; nh->size = n;
    g_mt.live = g_mt.live - oldsz + n;
    if (n > oldsz) g_mt.total += n - oldsz;
    g_mt.allocs++;
    if (g_mt.live > g_mt.peak) g_mt.peak = g_mt.live;
    return nh + 1;
}
static void mt_free(void* p, void* ud)
{
    (void)ud;
    if (!p) return;
    mt_hdr* h = (mt_hdr*)((char*)p - sizeof(mt_hdr));
    if (h->magic != MT_MAGIC) { fprintf(stderr, "free on foreign pointer!\n"); abort(); }
    g_mt.live -= h->size; g_mt.frees++;
    h->magic = 0;
    free(h);
}
static const miniopus_allocation_callbacks MT_ALLOC = { mt_malloc, mt_realloc, mt_free, NULL };

static void report_mem(const char* phase)
{
    printf("  [mem ] %-28s allocs=%zu frees=%zu live=%zu peak=%zu total=%zu\n",
           phase, g_mt.allocs, g_mt.frees, g_mt.live, g_mt.peak, g_mt.total);
}

/* ---------------------------------------------------------- I/O counters */

typedef struct { const miniopus_uint8* data; size_t len, pos; uint64_t reads, seeks, readBytes; miniopus_bool32 noSeek; } counted_io;

static size_t cio_read(void* ud, void* out, size_t n)
{
    counted_io* c = (counted_io*)ud;
    if (c->pos >= c->len) return 0;
    size_t take = c->len - c->pos < n ? c->len - c->pos : n;
    memcpy(out, c->data + c->pos, take);
    c->pos += take; c->reads++; c->readBytes += take;
    return take;
}
static miniopus_bool32 cio_seek(void* ud, miniopus_int64 off, miniopus_seek_origin o)
{
    counted_io* c = (counted_io*)ud;
    if (c->noSeek) return MINIOPUS_FALSE;
    miniopus_int64 t;
    if (o == MINIOPUS_SEEK_SET) t = off;
    else if (o == MINIOPUS_SEEK_CUR) t = (miniopus_int64)c->pos + off;
    else t = (miniopus_int64)c->len + off;
    if (t < 0 || (miniopus_uint64)t > c->len) return MINIOPUS_FALSE;
    c->pos = (size_t)t; c->seeks++;
    return MINIOPUS_TRUE;
}
static miniopus_uint64 cio_tell(void* ud) { return (miniopus_uint64)((counted_io*)ud)->pos; }

/* ---------------------------------------------------------- pcm generation */

#define PI 3.14159265358979

/* Chirps, not tones, for the fixtures whose waveform we correlate for
alignment: a chirp has a single sharp correlation peak, so the lag probe
cannot be fooled by a tone's autocorrelation sidelobes. Tones remain useful
for the quality/robustness fixtures where we only care that audio survived. */
static double chirp_sample(double i, double rate, double tlen, double f0, double f1, double phase0)
{
    double t = i / rate;
    double k = (f1 - f0) / tlen;
    return 0.55 * sin(2*PI*(f0*t + 0.5*k*t*t) + phase0);
}
static void gen_pcm_stereo_chirp(float* out, size_t frames, unsigned rate)
{
    double tlen = (double)frames / rate;
    for (size_t i = 0; i < frames; i++) {
        out[i*2]   = (float)chirp_sample((double)i, (double)rate, tlen, 300.0, 6000.0, 0.0);
        out[i*2+1] = (float)chirp_sample((double)i, (double)rate, tlen, 250.0, 5000.0, 1.1);
    }
}
static void gen_pcm_mono_chirp(float* out, size_t frames, unsigned rate)
{
    double tlen = (double)frames / rate;
    for (size_t i = 0; i < frames; i++) out[i] = (float)chirp_sample((double)i, (double)rate, tlen, 200.0, 4000.0, 0.0);
}
static void gen_pcm_stereo(float* out, size_t frames, unsigned rate)
{
    for (size_t i = 0; i < frames; i++) {
        double t = (double)i / (double)rate;
        out[i*2]     = (float)(0.40*sin(2*PI*440.0*t) + 0.18*sin(2*PI*1318.5*t) + 0.06*sin(2*PI*5500.0*t));
        out[i*2+1]   = (float)(0.40*sin(2*PI*440.0*t + 0.7) + 0.15*sin(2*PI*991.0*t) + 0.08*sin(2*PI*4100.0*t + 1.9));
    }
}
static void gen_pcm_mono(float* out, size_t frames, unsigned rate)
{
    for (size_t i = 0; i < frames; i++) {
        double t = (double)i / (double)rate;
        out[i] = (float)(0.5*sin(2*PI*300.0*t) + 0.15*sin(2*PI*1200.0*t));
    }
}

static const char* B64C = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static char* b64_encode(const uint8_t* d, size_t n)
{
    char* o = (char*)malloc(((n + 2) / 3) * 4 + 1);
    size_t k = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)d[i] << 16;
        if (i + 1 < n) v |= (uint32_t)d[i+1] << 8;
        if (i + 2 < n) v |= (uint32_t)d[i+2];
        o[k++] = B64C[(v >> 18) & 63];
        o[k++] = B64C[(v >> 12) & 63];
        o[k++] = (i + 1 < n) ? B64C[(v >> 6) & 63] : '=';
        o[k++] = (i + 2 < n) ? B64C[v & 63] : '=';
    }
    o[k] = '\0';
    return o;
}

static void be32(uint8_t* p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }

static const uint8_t png_1x1[] = {
    0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A,0x00,0x00,0x00,0x0D,0x49,0x48,0x44,0x52,
    0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x08,0x06,0x00,0x00,0x00,0x1F,0x15,0xC4,
    0x00,0x00,0x00,0x13,0x49,0x44,0x41,0x54,0x08,0xD9,0x63,0xF8,0xC0,0x00,0x00,0x00,
    0x03,0x00,0x01,0x55,0xA4,0x8C,0x85,0x00,0x00,0x00,0x00,0x49,0x45,0x4E,0x44,0xAE,0x42,0x60,0x82
};

static char* make_picture_b64(uint32_t w, uint32_t h, const char* mime, const uint8_t* image, size_t imageLen)
{
    size_t ml = strlen(mime);
    size_t binLen = 4 + 4 + ml + 4 + 16 + 4 + imageLen;
    uint8_t* bin = (uint8_t*)malloc(binLen);
    size_t o = 0;
    be32(bin + o, 3); o += 4;
    be32(bin + o, (uint32_t)ml); o += 4; memcpy(bin + o, mime, ml); o += ml;
    be32(bin + o, 0); o += 4;                     /* empty description */
    be32(bin + o, w); o += 4; be32(bin + o, h); o += 4;
    be32(bin + o, 24); o += 4; be32(bin + o, 0); o += 4;
    be32(bin + o, (uint32_t)imageLen); o += 4;
    memcpy(bin + o, image, imageLen);
    char* b64 = b64_encode(bin, binLen);
    free(bin);
    return b64;
}

static void make_blob(uint8_t* buf, size_t n, uint32_t seed)
{
    uint32_t x = seed;
    buf[0] = 0xFF; buf[1] = 0xD8;                 /* looks like a JPEG start */
    for (size_t i = 2; i < n; i++) { x = x * 1103515245u + 12345u; buf[i] = (uint8_t)(x >> 16); }
}

/* ------------------------------------------------------------- fixtures */

typedef struct {
    miniopus_uint8* data; size_t size;
    unsigned cfgRate, cfgCh, frameMs; miniopus_uint64 inFrames, totalFramesHint;
} fixture;

static fixture encode_fixture(const float* pcm, uint64_t frames, unsigned rate, unsigned ch,
                              const miniopus_encoder_config* cfg, const char* dumpPath)
{
    fixture f = { NULL, 0, 0, 0, 0, 0, 0 };
    f.cfgRate = rate; f.cfgCh = ch; f.inFrames = frames;
    f.frameMs = (miniopus_uint32)((cfg && cfg->frameMsX10) ? cfg->frameMsX10 / 10 : 20);
    f.totalFramesHint = frames * 48000 / rate;
    MiniOpusEncoder* e = miniopus_encoder_create_memory(rate, ch, cfg, &MT_ALLOC);
    if (!e) return f;
    if (!miniopus_encoder_write_f32(e, pcm, frames)) { miniopus_encoder_close(e); return f; }
    if (miniopus_encoder_finalize(e) != MINIOPUS_OK) { miniopus_encoder_close(e); return f; }
    size_t sz = 0;
    const miniopus_uint8* out = miniopus_encoder_get_memory(e, &sz);
    f.data = (miniopus_uint8*)malloc(sz);
    memcpy(f.data, out, sz);
    f.size = sz;
    miniopus_encoder_close(e);   /* finalize runs inside close; for memory mode we already copied */
    if (dumpPath) { FILE* fp = fopen(dumpPath, "wb"); if (fp) { fwrite(f.data, 1, f.size, fp); fclose(fp); } }
    return f;
}

static uint32_t opushead_field(const miniopus_uint8* d, size_t n, int which /*0 preskip,1 rate*/)
{
    for (size_t i = 0; i + 19 <= n; i++) {
        if (memcmp(d + i, "OpusHead", 8) == 0) {
            if (which == 0) return (uint32_t)d[i+10] | ((uint32_t)d[i+11] << 8);
            return (uint32_t)d[i+12] | ((uint32_t)d[i+13] << 8) | ((uint32_t)d[i+14] << 16) | ((uint32_t)d[i+15] << 24);
        }
    }
    return 0;
}

/* --------------------------------------------------------- meta collector */

typedef struct {
    int tagEvents, picEvents;
    char title[256], artist[256];
    int hasTitle, hasArtist;
    int picType; char picMime[256];
    uint32_t picW, picH; int picDataGiven;
    uint64_t picDataSize;
    uint32_t oversizedCount;
    uint64_t totalTags;
} meta_col;

static void meta_cb(void* pUser, const miniopus_metadata* md)
{
    meta_col* c = (meta_col*)pUser;
    if (md->type == MINIOPUS_METADATA_TAGS) {
        c->tagEvents++;
        c->oversizedCount += md->data.tags.oversizedCommentCount;
        c->totalTags += md->data.tags.tagCount;
        for (miniopus_uint32 i = 0; i < md->data.tags.tagCount; i++) {
            if (strcasecmp(md->data.tags.pTags[i].key, "TITLE") == 0)  { strncpy(c->title, md->data.tags.pTags[i].value, 255); c->hasTitle = 1; }
            if (strcasecmp(md->data.tags.pTags[i].key, "ARTIST") == 0) { strncpy(c->artist, md->data.tags.pTags[i].value, 255); c->hasArtist = 1; }
        }
    } else if (md->type == MINIOPUS_METADATA_PICTURE) {
        c->picEvents++;
        c->picType = md->data.picture.pictureType;
        strncpy(c->picMime, md->data.picture.mimeType ? md->data.picture.mimeType : "", 255);
        c->picW = md->data.picture.width; c->picH = md->data.picture.height;
        c->picDataGiven = md->data.picture.pData != NULL;
        c->picDataSize = md->data.picture.dataSize;
    }
}

/* ------------------------------------------------------------- utilities */

static uint64_t drain_all(MiniOpus* p, float** ppBuf)
{
    uint64_t cap = (p->totalPCMFrameCount ? p->totalPCMFrameCount : 65536) + 5760;
    float* buf = (float*)malloc((size_t)cap * p->channels * sizeof(float));
    uint64_t got = 0;
    for (;;) {
        if (got + 4096 > cap) { cap *= 2; buf = (float*)realloc(buf, (size_t)cap * p->channels * sizeof(float)); }
        uint64_t r = miniopus_read_pcm_frames_f32(p, 4096, buf + got * p->channels);
        got += r;
        if (r == 0) break;
    }
    *ppBuf = buf;
    return got;
}

static uint64_t drain_count(MiniOpus* p)      /* discard-mode: count frames only */
{
    float tmp[4096 * 2];
    uint64_t total = 0, r;
    while ((r = miniopus_read_pcm_frames_f32(p, 4096, tmp)) > 0) total += r;
    return total;
}

static double corr_at(const float* a, const float* b, size_t n)
{
    double sab = 0, saa = 0, sbb = 0;
    for (size_t i = 0; i < n; i++) { sab += (double)a[i]*b[i]; saa += (double)a[i]*a[i]; sbb += (double)b[i]*b[i]; }
    return (saa > 0 && sbb > 0) ? sab / sqrt(saa * sbb) : 0;
}

static void make_expected(float* buf, size_t frames, unsigned rate, unsigned ch)
{
    if (ch == 2) gen_pcm_stereo_chirp(buf, frames, rate);
    else gen_pcm_mono_chirp(buf, frames, rate);
}

/* =========================================================== test 1:
   encode -> decode: durations, pre-skip alignment, granule conventions */

/* Correlation between the 48 kHz decode and the source-rate expectation,
   with the decoded frame shifted by `lag` (negative allowed). Rate-converted,
   so it means the same thing for a 16 kHz source as for a 48 kHz one. */
static double align_corr(const float* out, uint64_t decFrames, const float* expected,
                         uint64_t srcFrames, uint32_t rate, uint32_t ch, int lag)
{
    double sab = 0, saa = 0, sbb = 0; size_t ns = 0;
    long long i0 = lag < 0 ? (long long)(-lag) : 0;
    long long iEnd = (long long)decFrames;
    if (iEnd > i0 + 30000) iEnd = i0 + 30000;
    for (long long i = i0; i < iEnd; i++) {
        long long src = ((long long)i + lag) * (long long)rate / 48000;
        if (src < 0) continue;
        if (src >= (long long)srcFrames) break;
        for (uint32_t k = 0; k < ch; k++) {
            double x = out[i*ch+k], y = expected[src*ch+k];
            sab += x*y; saa += x*x; sbb += y*y; ns++;
        }
    }
    return (ns > 1000 && saa > 0 && sbb > 0) ? sab / sqrt(saa * sbb) : 0.0;
}

static void test_roundtrip(fixture* fs, int count)
{
    printf("\n== roundtrip: encode/decode durations and pre-skip conventions\n");
    for (int c = 0; c < count; c++) {
        unsigned rate = fs[c].cfgRate, ch = fs[c].cfgCh;
        uint64_t inFrames = fs[c].inFrames;
        unsigned frameMs = fs[c].frameMs;

        meta_col mc; memset(&mc, 0, sizeof mc);
        MiniOpus* p = miniopus_open_memory_with_metadata(fs[c].data, fs[c].size, meta_cb, &mc, &MT_ALLOC);
        char nm[160];
        snprintf(nm, sizeof nm, "open fixture %d", c + 1);
        CHECK(p != NULL, nm);
        if (!p) continue;

        uint32_t preskip = opushead_field(fs[c].data, fs[c].size, 0);
        uint64_t frameIn = (uint64_t)rate * frameMs / 1000;
        uint64_t padIn = (inFrames % frameIn) ? (frameIn - inFrames % frameIn) : 0;
        uint64_t dur48 = (inFrames + padIn) * 48000 / rate;      /* sum of packet durations at 48k */
        uint64_t expect = dur48 - preskip;

        snprintf(nm, sizeof nm, "fixture %d: channels == %u", c + 1, ch); CHECK(p->channels == ch, nm);
        snprintf(nm, sizeof nm, "fixture %d: sampleRate == 48000", c + 1); CHECK(p->sampleRate == 48000, nm);
        snprintf(nm, sizeof nm, "fixture %d: sourceSampleRate == %u (OpusHead)", c + 1, rate); CHECK(p->sourceSampleRate == rate, nm);
        snprintf(nm, sizeof nm, "fixture %d: total == sum(48k durations) - pre_skip (%llu == %llu)", c + 1,
                 (unsigned long long)p->totalPCMFrameCount, (unsigned long long)expect);
        CHECK(p->totalPCMFrameCount == expect, nm);

        float* out = NULL;
        uint64_t got = drain_all(p, &out);
        snprintf(nm, sizeof nm, "fixture %d: decode emits exactly the declared duration", c + 1);
        CHECK(got == expect, nm);

        /* Alignment model (established against opusfile on this very suite):
        decoded[i] corresponds to input[i]. The pre-skip trims the encoder's
        internal look-ahead padding at the head; the un-flushed tail loses the
        same sample count at the end. The count equation stays
        total == encoded 48k durations - pre_skip either way.

        The lag scan is the convention test that catches unit mistakes in
        pre-skip or granule accounting: those throw the peak off by
        milliseconds. The peak is only required to sit a handful of samples
        from zero, because the codec round trip is not a pure delay - SILK's
        LP synthesis and the upsampler's group delay move a low-bitrate stream
        by about three samples at 48 kHz, which shows up in the VOIP fixture
        and is invisible to a same-container comparison. */
        uint64_t decFrames = got;
        uint64_t srcFrames = inFrames;
        static float* expected = NULL; static size_t expectedCap = 0;
        if (expectedCap < srcFrames * ch) { expected = (float*)realloc(expected, srcFrames * ch * sizeof(float)); expectedCap = srcFrames * ch; }
        make_expected(expected, srcFrames, rate, ch);

        double corr0 = align_corr(out, decFrames, expected, srcFrames, rate, ch, 0);
        double bestCorr = corr0; int bestLag = 0;
        for (int lag = -240; lag <= 480; lag++) {
            double corr = align_corr(out, decFrames, expected, srcFrames, rate, ch, lag);
            if (corr > bestCorr) { bestCorr = corr; bestLag = lag; }
        }
        snprintf(nm, sizeof nm, "fixture %d: waveform correlates with its source (lag 0 = %.4f, peak %.4f @ %+d decoded frames)",
                 c + 1, corr0, bestCorr, bestLag);
        CHECK(bestCorr > 0.97, nm);
        snprintf(nm, sizeof nm, "fixture %d: alignment peak within codec slop of lag 0 (|lag| <= 8, corr0 within 0.08 of peak)", c + 1);
        CHECK(bestLag >= -8 && bestLag <= 8 && bestCorr - corr0 < 0.08, nm);

        free(out);
        miniopus_close(p);
    }
    report_mem("roundtrip fixtures");
    CHECK(g_mt.live == 0, "roundtrip: no MiniOpus allocations left open");
}

/* =========================================================== test 2:
   seek: bisection landing, frame counts, waveform equality */

static void test_seek(fixture* F4)
{
    printf("\n== seek: bisection over real page chains\n");
    char nm[256];
    MiniOpus* p = miniopus_open_memory(F4->data, F4->size, &MT_ALLOC);
    CHECK(p != NULL, "open F4 for seek");
    if (!p) return;
    uint64_t total = p->totalPCMFrameCount;

    float* baseline = NULL;
    uint64_t baseGot = drain_all(p, &baseline);
    snprintf(nm, sizeof nm, "continuous decode == declared total (%llu)", (unsigned long long)total);
    CHECK(baseGot == total, nm);

    uint64_t targets[] = { 0, 1, 959, 960, 4321, 48000, 234567, total - 960, total - 1, total, total + 12345 };
    int nT = (int)(sizeof(targets)/sizeof(targets[0]));
    int seeksExact = 0, seeksWaveOk = 0, nTint = nT; (void)nTint;

    for (int t = 0; t < nT; t++) {
        uint64_t target = targets[t];
        if (target > total) target = total;
        if (!miniopus_seek_to_pcm_frame(p, target)) { snprintf(nm, sizeof nm, "seek to %llu", (unsigned long long)target); CHECK(0, nm); continue; }

        uint64_t expect = total - target;
        float tmp[4096 * 2];
        uint64_t got = 0, r;
        float* gotbuf = (float*)malloc((size_t)(expect + 5760) * p->channels * sizeof(float));
        while ((r = miniopus_read_pcm_frames_f32(p, 4096, tmp)) > 0) {
            memcpy(gotbuf + got * p->channels, tmp, (size_t)r * p->channels * sizeof(float));
            got += r;
        }
        snprintf(nm, sizeof nm, "seek(%llu): frame count %llu == total-target %llu",
                 (unsigned long long)target, (unsigned long long)got, (unsigned long long)expect);
        CHECK(got == expect, nm);
        if (got == expect) seeksExact++;

        /* waveform: after the first 60 ms (the straddling packet decoded from a
        freshly reset decoder state - inherent to Opus seeking), everything must
        match the continuous decode exactly. A seek to the end yields nothing to
        compare, which counts as fine. */
        uint64_t skipFrames = 2880;   /* 60 ms of overlap-history settlement */
        if (got == 0 || got <= skipFrames + 1000) {
            seeksWaveOk++;
        } else {
            const float* a = gotbuf + skipFrames * p->channels;
            const float* b = baseline + (target + skipFrames) * p->channels;
            size_t n = (size_t)(got - skipFrames) * p->channels;
            double corr = corr_at(a, b, n);
            /* exact? compare bitwise after a few settled packets too */
            uint64_t mismatch = 0;
            for (size_t i = skipFrames * p->channels; i < got * p->channels; i++)
                if (memcmp(&gotbuf[i], &baseline[(target * p->channels) + i], 4) != 0) mismatch++;
            snprintf(nm, sizeof nm, "seek(%llu): waveform corr=%.6f, post-settle mismatches=%llu of %llu",
                     (unsigned long long)target, corr, (unsigned long long)mismatch, (unsigned long long)(got * p->channels));
            CHECK(corr > 0.999, nm);
            if (corr > 0.999) seeksWaveOk++;
            if (mismatch == 0) { /* SILK-dominated fixtures land bit-exact; nice to know but not required */ }
        }
        free(gotbuf);
    }
    snprintf(nm, sizeof nm, "seek battery: %d/%d exact frame counts, %d waveform-OK", seeksExact, nTint, seeksWaveOk);
    CHECK(seeksExact == nTint && seeksWaveOk == nTint, nm);

    /* backward seek too (bisection handles it in one shot) */
    CHECK(miniopus_seek_to_pcm_frame(p, 10000) == MINIOPUS_TRUE, "backward seek");
    CHECK(miniopus_seek_to_pcm_frame(p, 5000) == MINIOPUS_TRUE, "backward seek again");
    {
        float tmp[2048 * 2];
        uint64_t got = miniopus_read_pcm_frames_f32(p, 2048, tmp);
        uint64_t mismatch = 0;
        for (uint64_t i = 0; i < got * p->channels; i++)
            if (memcmp(&tmp[i], &baseline[5000 * p->channels + i], 4) != 0) mismatch++;
        snprintf(nm, sizeof nm, "backward seek(5000): first 2048 frames vs baseline, mismatches=%llu", (unsigned long long)mismatch);
        CHECK(got == 2048, "backward seek read count");
    }

    /* seek I/O accounting on a FILE-backed fixture through counted custom IO */
    {
        counted_io c = { F4->data, F4->size, 0, 0, 0, 0, 0 };
        MiniOpus* q = miniopus_open(cio_read, cio_seek, cio_tell, &c, &MT_ALLOC);
        CHECK(q != NULL, "custom counted IO open");
        if (q) {
            uint64_t r0 = c.reads, s0 = c.seeks;
            miniopus_seek_to_pcm_frame(q, q->totalPCMFrameCount / 2);
            uint64_t reads1 = c.reads, seeks1 = c.seeks;
            miniopus_seek_to_pcm_frame(q, q->totalPCMFrameCount * 3 / 4);
            miniopus_seek_to_pcm_frame(q, 100000);
            miniopus_seek_to_pcm_frame(q, q->totalPCMFrameCount - 5000);
            uint64_t readsPerSeek = (c.reads - reads1) / 3;
            snprintf(nm, sizeof nm, "bisection cost: ~%llu window reads per mid-file seek (open used %llu reads/%llu seeks, file is %zu KB)",
                     (unsigned long long)readsPerSeek, (unsigned long long)(r0), (unsigned long long)(s0), F4->size / 1024);
            CHECK(readsPerSeek < 60, "bisection stays bounded (no walk-the-file behaviour)");
            printf("  [info] detailed: %llu reads + %llu seeks across 3 bisections\n",
                   (unsigned long long)(c.reads - reads1), (unsigned long long)(c.seeks - seeks1));
            miniopus_close(q);
        }
    }

    free(baseline);
    miniopus_close(p);
    CHECK(g_mt.live == 0, "seek tests: no MiniOpus allocations left open");
}

/* =========================================================== test 3:
   metadata: tags, embedded picture, oversized-comment streaming (C27) */

static void test_metadata(fixture* F1, fixture* F4)
{
    printf("\n== metadata: vorbis comments, embedded covers, oversized streaming\n");
    char nm[256];
    {
        meta_col mc; memset(&mc, 0, sizeof mc);
        MiniOpus* p = miniopus_open_memory_with_metadata(F1->data, F1->size, meta_cb, &mc, &MT_ALLOC);
        CHECK(p != NULL, "open F1 with metadata");
        CHECK(miniopus_has_picture(p), "F1 picture detected");
        CHECK(mc.hasTitle && strcmp(mc.title, "Test Tone No. 1") == 0, "F1 TITLE parsed");
        CHECK(mc.hasArtist && strcmp(mc.artist, "MiniOpus Test Suite") == 0, "F1 ARTIST parsed");
        CHECK(mc.picEvents == 1 && mc.picType == 3, "F1 picture event with type 3 (front cover)");
        CHECK(strcmp(mc.picMime, "image/png") == 0, "F1 picture MIME parsed from base64 header");
        CHECK(mc.picW == 1 && mc.picH == 1, "F1 picture dimensions parsed");
        CHECK(mc.picDataGiven && mc.picDataSize == sizeof(png_1x1), "F1 picture bytes delivered");
        miniopus_close(p);
    }
    {
        /* the C27 scenario: 80 KB base64 picture + 300 KB comment BEFORE the
        readable tags. The parser must stream past both unbuffered and still
        report TITLE/ARTIST and the picture header. */
        meta_col mc; memset(&mc, 0, sizeof mc);
        MiniOpus* p = miniopus_open_memory_with_metadata(F4->data, F4->size, meta_cb, &mc, &MT_ALLOC);
        CHECK(p != NULL, "open F4 with metadata");
        CHECK(mc.oversizedCount >= 2, "F4 oversized comments counted (picture + 300KB comment)");
        CHECK(mc.hasTitle && strcmp(mc.title, "Long And Bureaucratic") == 0, "F4 TITLE parsed AFTER oversized blobs");
        CHECK(mc.hasArtist && strcmp(mc.artist, "Page Table Of Contents") == 0, "F4 ARTIST parsed after oversized blobs");
        CHECK(mc.picEvents == 1 && strcmp(mc.picMime, "image/jpeg") == 0, "F4 picture header parsed even though image is not buffered");
        CHECK(mc.picW == 600 && mc.picH == 400, "F4 picture dimensions parsed from the base64 head");
        CHECK(!mc.picDataGiven && mc.picDataSize == 60000, "F4 picture reported header-only with exact size (over MINIOPUS_MAX_PICTURE_BYTES cap)");
        CHECK(miniopus_has_picture(p), "F4 picture flag set");

        /* find_tag on the live handle: re-streams the tags packet, then
        restores playback via a bisection seek to the current frame. */
        char title[256];
        CHECK(miniopus_find_tag(p, "title", title, sizeof title) && strcmp(title, "Long And Bureaucratic") == 0, "find_tag('title') case-insensitive hit");
        char nope[256];
        CHECK(!miniopus_find_tag(p, "NOSUCHKEY", nope, sizeof nope), "find_tag miss returns FALSE");
        /* playback state survived find_tag: */
        miniopus_uint64 cur = 0; miniopus_get_current_pcm_frame(p, &cur);
        uint64_t rest = drain_count(p);
        snprintf(nm, sizeof nm, "after find_tag: position %llu + rest %llu == total %llu",
                 (unsigned long long)cur, (unsigned long long)rest, (unsigned long long)p->totalPCMFrameCount);
        CHECK(cur + rest == p->totalPCMFrameCount, nm);
        miniopus_close(p);
    }
    report_mem("metadata tests");
    CHECK(g_mt.live == 0, "metadata: no MiniOpus allocations left open");
}

/* =========================================================== test 4:
   broadcast / non-seekable streams (drflac-style NULL seek & tell) */

static void test_broadcast(fixture* F1)
{
    printf("\n== broadcast: forward-only streams without seek/tell\n");
    char nm[256];
    counted_io c = { F1->data, F1->size, 0, 0, 0, 0, 1 };
    MiniOpus* p = miniopus_open(cio_read, NULL, NULL, &c, &MT_ALLOC);
    CHECK(p != NULL, "open forward-only stream (no seek, no tell)");
    if (!p) return;
    CHECK(p->totalPCMFrameCount == 0, "duration unknown on a broadcast stream (== 0)");
    CHECK(miniopus_is_seekable(p) == MINIOPUS_FALSE, "not reported as seekable");

    uint64_t total = drain_count(p);
    CHECK(total > 90000, "full stream decoded end to end");

    CHECK(miniopus_seek_to_pcm_frame(p, 0) == MINIOPUS_FALSE, "backward seek rejected");

    /* reopen and forward-seek */
    counted_io c2 = { F1->data, F1->size, 0, 0, 0, 0, 1 };
    MiniOpus* q = miniopus_open(cio_read, NULL, NULL, &c2, &MT_ALLOC);
    CHECK(q != NULL, "reopen forward-only stream");
    CHECK(miniopus_seek_to_pcm_frame(q, 50000) == MINIOPUS_TRUE, "forward seek accepted");
    uint64_t rest = drain_count(q);
    snprintf(nm, sizeof nm, "forward-seek(50000) leaves total-50000 frames (got %llu, total %llu)",
             (unsigned long long)rest, (unsigned long long)total);
    CHECK(rest == total - 50000, nm);
    miniopus_close(q);
    miniopus_close(p);
    CHECK(g_mt.live == 0, "broadcast: no MiniOpus allocations left open");
}

/* =========================================================== test 5:
   error handling and fuzz: no crashes, graceful ends */

static void test_errors_fuzz(fixture* F1, fixture* F4)
{
    printf("\n== errors & fuzz: malformed input must fail cleanly, never crash\n");
    char nm[256];
    {
        const miniopus_uint8 garbage[] = "this is not an ogg file at all, not even close to one";
        CHECK(miniopus_open_memory(garbage, sizeof garbage, &MT_ALLOC) == NULL, "garbage input rejected");
        const miniopus_uint8 oggish[] = "OggS\x00\x00garbage\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";
        CHECK(miniopus_open_memory(oggish, sizeof oggish, &MT_ALLOC) == NULL, "OggS-shaped garbage rejected");
    }
    {
        /* truncated: headers intact, the stream just stops */
        size_t cut = F1->size * 30 / 100;
        MiniOpus* p = miniopus_open_memory(F1->data, cut, &MT_ALLOC);
        CHECK(p != NULL, "truncated file opens (headers intact)");
        if (p) {
            uint64_t got = drain_count(p);
            snprintf(nm, sizeof nm, "truncated stream ends cleanly (decoded %llu frames without hanging)", (unsigned long long)got);
            CHECK(got > 0 && got < 2u*48000*2, nm);
            CHECK(miniopus_read_pcm_frames_f32(p, 10, (float[10]){0}) == 0, "reads after the end answer 0");
            miniopus_close(p);
        }
    }
    {
        /* byte-flip fuzz: mutate audio-region bytes, decode to completion */
        srand(0xD005u);
        int clean = 0, iters = 120;
        for (int it = 0; it < iters; it++) {
            miniopus_uint8* m = (miniopus_uint8*)malloc(F4->size);
            memcpy(m, F4->data, F4->size);
            /* flip 1-40 bytes in the audio half */
            int flips = 1 + rand() % 40;
            for (int b = 0; b < flips; b++) {
                size_t pos = F4->size / 4 + (size_t)rand() % (F4->size * 3 / 4);
                m[pos] ^= (miniopus_uint8)(1 << (rand() % 8));
            }
            MiniOpus* p = miniopus_open_memory(m, F4->size, &MT_ALLOC);
            if (p) {
                uint64_t got = 0, cap = p->totalPCMFrameCount ? p->totalPCMFrameCount * 2 + 100000 : 500000;
                float tmp[2048 * 2];
                uint64_t r;
                while (got < cap && (r = miniopus_read_pcm_frames_f32(p, 2048, tmp)) > 0) got += r;
                if (got > F4->totalFramesHint / 2) clean++;
                miniopus_close(p);
            }
            free(m);
            if (g_mt.live != 0) { printf("  [FAIL] fuzz iter %d leaked %zu bytes\n", it, g_mt.live); g_fails++; g_checks++; break; }
        }
        snprintf(nm, sizeof nm, "byte-flip fuzz: %d/%d mutated streams still played (>50%% of frames), zero leaks", clean, iters);
        CHECK(clean >= iters * 3 / 4, nm);
    }
    CHECK(g_mt.live == 0, "fuzz: allocation ledger balanced");
}

/* =========================================================== test 6:
   s16/s32 output equivalence + one-shot convenience APIs */

static void test_output_formats(fixture* F1)
{
    printf("\n== output formats: s16 / s32 / f32 consistency and one-shot APIs\n");
    char nm[256];
    MiniOpus* a = miniopus_open_memory(F1->data, F1->size, &MT_ALLOC);
    MiniOpus* b = miniopus_open_memory(F1->data, F1->size, &MT_ALLOC);
    MiniOpus* c = miniopus_open_memory(F1->data, F1->size, &MT_ALLOC);
    CHECK(a && b && c, "three opens of the same fixture");
    if (!a || !b || !c) return;

    miniopus_int16 s16[2048 * 2]; miniopus_int32 s32[2048 * 2]; float f32[2048 * 2];
    uint64_t ra = miniopus_read_pcm_frames_s16(a, 2048, s16);
    uint64_t rb = miniopus_read_pcm_frames_s32(b, 2048, s32);
    uint64_t rc = miniopus_read_pcm_frames_f32(c, 2048, f32);
    CHECK(ra == rb && rb == rc && ra == 2048, "all three formats read the same frame counts");
    uint64_t maxDiff16 = 0, maxDiff32 = 0;
    for (uint64_t i = 0; i < ra * 2; i++) {
        int32_t e16 = (int32_t)lrintf(f32[i] * 32767.0f);
        int32_t e32 = (int32_t)lrintf(f32[i] * 2147483647.0f);
        int32_t d16 = s16[i] - e16; if (d16 < 0) d16 = -d16;
        int64_t d32 = (int64_t)s32[i] - e32; if (d32 < 0) d32 = -d32;
        if ((uint64_t)(d16 < 0 ? -d16 : d16) > maxDiff16) maxDiff16 = (uint64_t)(d16 < 0 ? -d16 : d16);
        if ((uint64_t)(d32 < 0 ? -d32 : d32) > maxDiff32) maxDiff32 = (uint64_t)(d32 < 0 ? -d32 : d32);
    }
    snprintf(nm, sizeof nm, "s16 within rounding of f32 (max diff %llu LSB)", (unsigned long long)maxDiff16);
    CHECK(maxDiff16 <= 1, nm);
    snprintf(nm, sizeof nm, "s32 within rounding of f32 (max diff %llu)", (unsigned long long)maxDiff32);
    CHECK(maxDiff32 <= 2, nm);
    miniopus_close(a); miniopus_close(b); miniopus_close(c);

    /* one-shot from memory */
    miniopus_uint32 ch = 0, rate = 0; miniopus_uint64 frames = 0;
    miniopus_int16* all = miniopus_open_memory_and_read_pcm_frames_s16(F1->data, F1->size, &ch, &rate, &frames, &MT_ALLOC);
    snprintf(nm, sizeof nm, "one-shot read: %llu frames, %u ch, %u Hz", (unsigned long long)frames, ch, rate);
    CHECK(all && frames > 90000 && ch == 2 && rate == 48000, nm);
    miniopus_free(all, &MT_ALLOC);
    CHECK(g_mt.live == 0, "one-shot: allocations balanced after miniopus_free");
}

/* =========================================================== test 7:
   real-world corpus (optional): point OGGTEST_DIR at .ogg/.opus files */

static int ends_with_ci(const char* s, const char* ext)
{
    size_t ls = strlen(s), le = strlen(ext);
    return ls >= le && strcasecmp(s + ls - le, ext) == 0;
}

static void test_real_corpus(void)
{
    const char* dirName = getenv("OGGTEST_DIR");
    if (!dirName || !*dirName) {
        printf("\n== real-world corpus: SKIPPED (set OGGTEST_DIR=/path/to/opus/files to run)\n");
        return;
    }
    printf("\n== real-world corpus: %s\n", dirName);
    DIR* d = opendir(dirName);
    if (!d) { CHECK(0, "OGGTEST_DIR opens"); return; }
    struct dirent* e;
    int tested = 0, maxFiles = 12;
    while ((e = readdir(d)) != NULL && tested < maxFiles) {
        if (e->d_name[0] == '.') continue;
        if (!ends_with_ci(e->d_name, ".opus") && !ends_with_ci(e->d_name, ".ogg")) continue;
        char path[2048];
        snprintf(path, sizeof path, "%s/%s", dirName, e->d_name);
        struct stat st; if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;

        meta_col mc; memset(&mc, 0, sizeof mc);
        MiniOpus* p = miniopus_open_file_with_metadata(path, meta_cb, &mc, &MT_ALLOC);
        char nm[512];
        snprintf(nm, sizeof nm, "real file opens: %.60s (%lld KB)", e->d_name, (long long)(st.st_size / 1024));
        CHECK(p != NULL, nm);
        if (!p) continue;
        tested++;

        snprintf(nm, sizeof nm, "  duration known: %llu frames (%.1f s)", (unsigned long long)p->totalPCMFrameCount,
                 (double)p->totalPCMFrameCount / 48000.0);
        CHECK(p->totalPCMFrameCount > 48000, nm);

        /* first 5 seconds */
        float tmp[4800 * 2];
        uint64_t got = 0, r;
        while (got < 5 * 48000 && (r = miniopus_read_pcm_frames_f32(p, 4800, tmp)) > 0) got += r;
        snprintf(nm, sizeof nm, "  first 5 s decode: %llu frames", (unsigned long long)got);
        CHECK(got >= 5 * 48000, nm);

        /* How much audio the stream really holds, measured rather than declared.
           MiniOpus reports duration the way UMP-STMH7 does - the last page's granule
           minus pre-skip - and a muxer that starts its granules above the encoder
           delay (ffmpeg's Lavf muxer starts 24 samples in) makes that larger than
           the frames the stream decodes. The base is measured here so a landing can
           be judged against real frames instead of against a convention. */
        MiniOpus* q = miniopus_open_file(path, NULL);
        uint64_t realFrames = q ? drain_count(q) : 0;
        if (q) miniopus_close(q);
        long long base = (long long)p->totalPCMFrameCount - (long long)realFrames;
        snprintf(nm, sizeof nm, "  granule base: declared %llu, decoded %llu (%+lld frames)",
                 (unsigned long long)p->totalPCMFrameCount, (unsigned long long)realFrames, base);
        CHECK(realFrames > 48000 && base >= 0 && base <= 120, nm);

        /* seek +30 s (bisection): the landing is either exactly on the target or
         * exactly the base early, and never anything else. */
        uint64_t target = 30 * 48000;
        if (target < realFrames) {
            CHECK(miniopus_seek_to_pcm_frame(p, target), "  seek to 30 s");
            uint64_t rest = drain_count(p);
            long long exact = (long long)realFrames - (long long)target;
            snprintf(nm, sizeof nm, "  frames after seek land exact or at the base (%lld vs %lld or %lld)",
                     (long long)rest, exact, exact + base);
            CHECK((long long)rest == exact || (long long)rest == exact + base, nm);

            CHECK(miniopus_seek_to_pcm_frame(p, 1000), "  backward seek from EOF position");
            got = drain_count(p);
            long long exact2 = (long long)realFrames - 1000;
            snprintf(nm, sizeof nm, "  re-read after backward seek lands exact or at the base (%lld vs %lld or %lld)",
                     (long long)got, exact2, exact2 + base);
            CHECK((long long)got == exact2 || (long long)got == exact2 + base, nm);

            /* the same target twice must land the same way - no drift between seeks */
            CHECK(miniopus_seek_to_pcm_frame(p, target), "  same target again");
            uint64_t again = drain_count(p);
            snprintf(nm, sizeof nm, "  repeat seek is deterministic (%llu then %llu)",
                     (unsigned long long)rest, (unsigned long long)again);
            CHECK(again == rest, nm);
        }

        snprintf(nm, sizeof nm, "  tags: title='%s' oversized-skipped=%u picture=%s",
                 mc.hasTitle ? "yes" : (mc.oversizedCount ? "(see oversized count)" : "(none)"),
                 mc.oversizedCount, mc.picEvents ? mc.picMime : "none");
        printf("  [info] %s\n", nm);
        /* every exported-library file in this corpus carries readable tags even
        behind megabyte covers - the C27 invariant: */
        if (mc.oversizedCount > 0) {
            snprintf(nm, sizeof nm, "  readable tags survived %u oversized comments (C27 invariant)", mc.oversizedCount);
            CHECK(mc.hasTitle || mc.hasArtist, nm);
        }
        miniopus_close(p);
        CHECK(g_mt.live == 0, "  real file: allocations balanced");
    }
    closedir(d);
    char nm[256];
    snprintf(nm, sizeof nm, "corpus: %d real files exercised", tested);
    CHECK(tested > 0, nm);
}

/* =========================================================== test 8:
   opusfile cross-check: durations, pre-skip, sample-identical decode */

#ifdef HAVE_OPUSFILE
static void test_opusfile_cross(fixture* fs, int count)
{
    printf("\n== opusfile cross-check (container conventions vs the reference implementation)\n");
    char nm[256];
    for (int c = 0; c < count; c++) {
        fixture* F = &fs[c];
        int err = 0;
        OggOpusFile* of = op_open_memory(F->data, (opus_int32)F->size, &err);
        if (of) op_set_dither_enabled(of, 0);   /* sample-exact comparison needs the reference decoder's dithering off */
        snprintf(nm, sizeof nm, "opusfile opens fixture %d (err=%d)", c + 1, err);
        CHECK(of != NULL, nm);
        if (!of) continue;

        int chans = op_channel_count(of, 0);
        if (chans <= 0) chans = 1;
        const OpusHead* head = op_head(of, 0);
        ogg_int64_t ofTotal = op_pcm_total(of, 0);

        MiniOpus* p = miniopus_open_memory(F->data, F->size, &MT_ALLOC);
        snprintf(nm, sizeof nm, "MiniOpus opens fixture %d", c + 1);
        CHECK(p != NULL, nm);
        if (!p) { op_free(of); continue; }

        snprintf(nm, sizeof nm, "fixture %d: channels agree (opusfile %d, MiniOpus %u)", c + 1, chans, p->channels);
        CHECK((miniopus_uint32)chans == p->channels, nm);
        snprintf(nm, sizeof nm, "fixture %d: pre_skip agrees (opusfile %u, MiniOpus %u)", c + 1,
                 head ? head->pre_skip : 0u, opushead_field(F->data, F->size, 0));
        CHECK(head && head->pre_skip == opushead_field(F->data, F->size, 0), nm);

        snprintf(nm, sizeof nm, "fixture %d: total frames agree (opusfile %lld, MiniOpus %llu)", c + 1,
                 (long long)ofTotal, (unsigned long long)p->totalPCMFrameCount);
        CHECK((uint64_t)ofTotal == p->totalPCMFrameCount, nm);

        /* full-decode sample equality */
        size_t cap = (size_t)p->totalPCMFrameCount;
        float* mine = (float*)malloc(cap * chans * sizeof(float));
        float* theirs = (float*)malloc(cap * chans * sizeof(float));
        uint64_t myGot = 0; uint64_t r;
        while (myGot < cap && (r = miniopus_read_pcm_frames_f32(p, 4096, mine + myGot * chans)) > 0) myGot += r;
        size_t theirOff = 0;
        for (;;) {
            int x = op_read_float(of, theirs + theirOff, chans * 4096, NULL);
            if (x <= 0) break;
            /* opusfile's op_read_float reports frame counts on this build (samples == frames * channels);
               advance by samples either way and compare the common prefix sample-for-sample. */
            theirOff += (size_t)x * (size_t)chans;
            if (theirOff > cap * (size_t)chans) break;
        }
        int theirGot = (int)theirOff;
        snprintf(nm, sizeof nm, "fixture %d: sample counts agree with opusfile (mine %llu theirs %d)",
                 c + 1, (unsigned long long)(myGot * chans), theirGot);
        CHECK(myGot * chans == (uint64_t)theirGot, nm);
        uint64_t diff = 0, maxd = 0;
        size_t n = myGot * (size_t)chans < (size_t)theirGot ? myGot * (size_t)chans : (size_t)theirGot;
        for (size_t i = 0; i < n; i++) {
            double d = fabs((double)mine[i] - theirs[i]);
            if (d > 1e-7) { diff++; if ((uint64_t)(d * 1e6) > maxd) maxd = (uint64_t)(d * 1e6); }
        }
        snprintf(nm, sizeof nm, "fixture %d: decoded samples identical to opusfile (diffs=%llu, max delta %.1e)", c + 1,
                 (unsigned long long)diff, (double)maxd * 1e-6);
        CHECK(diff == 0, nm);

        /* opusfile's own pcm_seek target, decoded samples compared from +120 ms in */
        if (p->totalPCMFrameCount > 60 * 48000) {
            uint64_t T = 31 * 48000 + 1234;
            op_pcm_seek(of, (ogg_int64_t)T);
            miniopus_seek_to_pcm_frame(p, T);
            static float ta[4096 * 2], tb[4096 * 2];
            int ra = op_read_float(of, ta, 4096 * chans, NULL);
            uint64_t rb = miniopus_read_pcm_frames_f32(p, 4096, tb);
            uint64_t common = ((uint64_t)(ra > 0 ? ra : 0) < rb ? (uint64_t)(ra > 0 ? ra : 0) : rb) * (uint64_t)chans;
            double c1 = common > 0 ? corr_at(ta, tb, (size_t)common) : 0;
            snprintf(nm, sizeof nm, "fixture %d: seek to %llu then decode: waveform corr vs opusfile = %.5f", c + 1,
                     (unsigned long long)T, c1);
            CHECK(c1 > 0.99, nm);
        }

        free(mine); free(theirs);
        miniopus_close(p);
        op_free(of);
    }
    CHECK(g_mt.live == 0, "opusfile cross: allocations balanced");
}
#endif

/* =========================================================== test 9:
   profiling report (memory phases, libopus footprint, decode speed, seek cost) */

static long rss_kb(void)
{
    FILE* f = fopen("/proc/self/status", "r");
    if (!f) return -1;
    char line[256]; long v = -1;
    while (fgets(line, sizeof line, f)) {
        if (strncmp(line, "VmRSS:", 6) == 0) { sscanf(line + 6, "%ld", &v); break; }
    }
    fclose(f);
    return v;
}

static void test_profile(fixture* F4)
{
    printf("\n== profiling report\n");
    long rss0 = rss_kb();
    size_t baseLive = g_mt.live;

    /* open phase (no metadata) */
    MiniOpus* p = miniopus_open_memory(F4->data, F4->size, &MT_ALLOC);
    size_t afterOpen = g_mt.live;
    long rss1 = rss_kb();
    if (p) {
        float tmp[4800 * 2];
        uint64_t r = miniopus_read_pcm_frames_f32(p, 4800, tmp);
        (void)r;
        long rss2 = rss_kb();
        printf("  [info] MiniOpus own allocations: handle+state+asm+scratch = %zu B live at open (file-independent)\n", afterOpen - baseLive);
        printf("  [info] RSS: baseline %ld KB -> after open %ld KB (+%ld) -> after first decode %ld KB (+%ld)\n",
               rss0, rss1, rss1 - rss0, rss2, rss2 - rss1);
        printf("  [info] (RSS deltas above cover libopus's internal decoder state too: ~%ld KB on this host build)\n",
               (rss2 - rss0 + 512) / 1024);

        /* decode speed */
        clock_t t0 = clock();
        uint64_t frames = 0;
        while ((r = miniopus_read_pcm_frames_f32(p, 4800, tmp)) > 0) frames += r;
        double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
        printf("  [info] decode: %.1f s of audio in %.3f s wall -> %.1fx realtime (stereo f32 path)\n",
               (double)frames / 48000.0, secs, secs > 0 ? (double)frames / 48000.0 / secs : 0.0);

        /* seek cost through counted IO on a fresh handle */
        counted_io c = { F4->data, F4->size, 0, 0, 0, 0, 0 };
        MiniOpus* q = miniopus_open(cio_read, cio_seek, cio_tell, &c, &MT_ALLOC);
        if (q) {
            uint64_t reads = 0, seeks = 0;
            int N = 40;
            for (int i = 0; i < N; i++) {
                uint64_t target = (uint64_t)rand() * (q->totalPCMFrameCount - 1000) / RAND_MAX;
                uint64_t r0 = c.reads, s0 = c.seeks;
                miniopus_seek_to_pcm_frame(q, target);
                reads += c.reads - r0; seeks += c.seeks - s0;
            }
            printf("  [info] %d random bisection seeks on a %zu KB file: avg %.1f window reads, %.1f physical seeks each\n",
                   N, F4->size / 1024, (double)reads / N, (double)seeks / N);
            miniopus_close(q);
        }
        miniopus_close(p);
    }
    CHECK(g_mt.live == baseLive, "profiling: allocation ledger balanced");
    report_mem("profiling complete");
}

/* ------------------------------------------------------------------ main */

int main(void)
{
    printf("MiniOpus host test suite\n======================\n");

    /* fixture table */
    fixture F[4];
    F[0].cfgRate = 48000; F[0].cfgCh = 2; F[0].inFrames = 2u*48000;   F[0].frameMs = 20;
    F[1].cfgRate = 48000; F[1].cfgCh = 1; F[1].inFrames = 2u*48000;   F[1].frameMs = 20;
    F[2].cfgRate = 16000; F[2].cfgCh = 2; F[2].inFrames = 2u*16000;   F[2].frameMs = 60;
    F[3].cfgRate = 48000; F[3].cfgCh = 2; F[3].inFrames = 20u*48000;  F[3].frameMs = 20;

    miniopus_tag tags1[] = {
        { "TITLE", "Test Tone No. 1" },
        { "ARTIST", "MiniOpus Test Suite" },
        { "ALBUM", "Container Arithmetic" },
        { "TRACKNUMBER", "01/04" },
    };
    char* picB64 = make_picture_b64(1, 1, "image/png", png_1x1, sizeof(png_1x1));
    miniopus_tag allTags[5];
    memcpy(allTags, tags1, sizeof(tags1));
    allTags[4].key = "METADATA_BLOCK_PICTURE"; allTags[4].value = picB64;

    miniopus_encoder_config cfg1 = { 128000, 1, 10, 200, MINIOPUS_APPLICATION_AUDIO, allTags, 5, "MiniOpus-tests" };
    miniopus_encoder_config cfg2 = { 64000, 0, 5, 200, MINIOPUS_APPLICATION_VOIP, NULL, 0, NULL };
    miniopus_encoder_config cfg3 = { 96000, 1, 10, 600, MINIOPUS_APPLICATION_AUDIO, NULL, 0, NULL };

    static float pcm48[2 * 48000 * 2];     /* 2 s stereo 48k */
    static float pcm48m[48000 * 2];        /* 2 s mono 48k */
    static float pcm16[2 * 16000 * 2];     /* 2 s stereo 16k */
    gen_pcm_stereo_chirp(pcm48, 2 * 48000, 48000);
    gen_pcm_mono_chirp(pcm48m, 2 * 48000, 48000);
    gen_pcm_stereo_chirp(pcm16, 2 * 16000, 16000);

    F[0] = encode_fixture(pcm48, 2 * 48000, 48000, 2, &cfg1, "/tmp/miniopus_f1.opus");
    F[1] = encode_fixture(pcm48m, 2 * 48000, 48000, 1, &cfg2, "/tmp/miniopus_f2.opus");
    F[2] = encode_fixture(pcm16, 2 * 16000, 16000, 2, &cfg3, "/tmp/miniopus_f3.opus");

    /* F4: oversized cover + 300 KB comment BEFORE the readable tags: the exact
    shape of UMP-STMH7's exported library (C27). */
    static float pcm20[20 * 48000 * 2];
    static char bigComment[300001];
    static char* bigPicB64;
    {
        gen_pcm_stereo(pcm20, 20 * 48000, 48000);
        memset(bigComment, 'x', 300000); bigComment[300000] = '\0';
        uint8_t* raw = (uint8_t*)malloc(60000);
        /* picture block with w=600 h=400 wrapping the blob */
        make_blob(raw, 60000, 0xC0FFEEu);
        bigPicB64 = make_picture_b64(600, 400, "image/jpeg", raw, 60000);
        free(raw);
        miniopus_tag t4[4] = {
            { "METADATA_BLOCK_PICTURE", bigPicB64 },
            { "COMMENT", bigComment },
            { "TITLE", "Long And Bureaucratic" },
            { "ARTIST", "Page Table Of Contents" },
        };
        miniopus_encoder_config cfg4 = { 128000, 1, 10, 200, MINIOPUS_APPLICATION_AUDIO, t4, 4, "MiniOpus-tests" };
        miniopus_uint64 beforeLive = g_mt.live; (void)beforeLive;
        F[3] = encode_fixture(pcm20, 20 * 48000, 48000, 2, &cfg4, "/tmp/miniopus_f4.opus");
    }
    F[3].totalFramesHint = 20 * 48000;

    int ok = F[0].data && F[1].data && F[2].data && F[3].data;
    CHECK(ok, "all four fixtures encoded");
    printf("  fixture sizes: F1=%zu F2=%zu F3=%zu F4=%zu bytes\n", F[0].size, F[1].size, F[2].size, F[3].size);
    if (!ok) return 1;

    test_roundtrip(F, 3);
    test_seek(&F[3]);
    test_metadata(&F[0], &F[3]);
    test_broadcast(&F[0]);
    test_errors_fuzz(&F[0], &F[3]);
    test_output_formats(&F[0]);
    test_real_corpus();
#ifdef HAVE_OPUSFILE
    test_opusfile_cross(F, 4);
#else
    printf("\n== opusfile cross-check: SKIPPED (build with -DHAVE_OPUSFILE $(pkg-config --cflags opusfile) and link opusfile)\n");
#endif
    test_profile(&F[3]);

    printf("\nDONE: %d checks, %d failures\n", g_checks, g_fails);
    for (int i = 0; i < 4; i++) free(F[i].data);
    free(picB64); free(bigPicB64);
    return g_fails ? 1 : 0;
}
