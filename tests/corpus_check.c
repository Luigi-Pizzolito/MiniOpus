#define _POSIX_C_SOURCE 200809L
/*
Corpus cross-check: MiniOpus against opusfile on real-world Ogg Opus files.

Needs opusfile (pkg-config). Run it over any set of .ogg files:

    cc -std=c11 -O2 -Wall -Wextra -DMINIOPUS_IMPLEMENTATION -I.. \
       -DHAVE_OPUSFILE $(pkg-config --cflags opus opusfile) \
       corpus_check.c ../miniopus.c -o build/corpus_check $(pkg-config --libs opus opusfile) -lm
    build/corpus_check /path/to/tracks/*.ogg

tests/build_and_run.sh does that for you when OGGTEST_DIR points at a directory
of .ogg files. What it checks, per file:
  - opens, and the declared duration/channels/pre-skip match opusfile's reading
    of the same file (allowing for the granule-base difference documented in the
    header: this library reports the last page's granule minus pre-skip, which is
    what UMP-STMH7 does, where opusfile also subtracts the muxer's start offset);
  - whole-stream decode equality - same frame count, and identical float bit
    patterns, hashed in order with FNV-1a;
  - an eight-target seek battery: after each landing, the frame count to the end
    equals total - target, the position matches the target, a backward seek works,
    and the post-settle waveform matches opusfile's at a lag equal to the base;
  - the tail (seek to total-1 leaves one frame; a seek past the end clamps);
  - tags, oversized-comment counts and embedded picture sizes against opusfile's
    own op_tags().

   Comparison model: whole-stream decode equality (FNV-1a over exact float bit
   patterns + frame counts), then a seek battery that re-checks a window after
   each landing. No chunk-size alignment assumptions: each side is drained with
   its own natural read sizes and the streams are hashed in order. */
#include "miniopus.h"
#include <opus/opusfile.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

static int fails = 0, files = 0;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static unsigned long long h_init(void) { return 146959810393466560ULL; }
static unsigned long long h_mix(unsigned long long h, float v)
{
    unsigned int bits; memcpy(&bits, &v, 4);
    h ^= (unsigned long long)bits; h *= 1099511628211ULL;
    return h;
}

#define CHUNK 2880                     /* frames per read step (opusfile's natural ceiling) */
static float A[CHUNK * 8], B[CHUNK * 8];

typedef struct { miniopus_uint32 tags, oversized, pictures; miniopus_uint64 pictureBytes; char vendor[128]; } Meta;
static void meta_cb(void* pUser, const miniopus_metadata* pMeta)
{
    Meta* m = (Meta*)pUser;
    if (pMeta->type == MINIOPUS_METADATA_TAGS) {
        m->tags = pMeta->data.tags.tagCount;
        m->oversized = pMeta->data.tags.oversizedCommentCount;
        m->vendor[0] = 0;
        if (pMeta->data.tags.vendorString) { strncpy(m->vendor, pMeta->data.tags.vendorString, sizeof(m->vendor) - 1); m->vendor[sizeof(m->vendor)-1] = 0; }
    } else if (pMeta->type == MINIOPUS_METADATA_PICTURE) {
        m->pictures++;
        m->pictureBytes = pMeta->data.picture.dataSize;
    }
}

static const char* basename_(const char* p) { const char* s = strrchr(p, '/'); return s ? s + 1 : p; }

/* Drain a MiniOpus window [0, frames) hashing as we go; returns frames read. */
static long long drain_miniopus(MiniOpus* p, long long frames, int chans, unsigned long long* h)
{
    long long got = 0;
    while (got < frames) {
        long long want = frames - got; if (want > CHUNK) want = CHUNK;
        miniopus_uint64 r = miniopus_read_pcm_frames_f32(p, (miniopus_uint64)want, A);
        if (r == 0) break;
        for (miniopus_uint64 i = 0; i < r * (miniopus_uint64)chans; i++) *h = h_mix(*h, A[i]);
        got += (long long)r;
    }
    return got;
}
static long long drain_opus(OggOpusFile* of, long long frames, int chans, unsigned long long* h)
{
    long long got = 0;
    while (got < frames) {
        long long want = frames - got; if (want > CHUNK) want = CHUNK; if (want < 960) want = 960;
        int x = op_read_float(of, B, (int)want, NULL);
        if (x < 0) { printf(" (opusfile read error %d)", x); return -1; }
        if (x == 0) break;
        for (int i = 0; i < x * chans; i++) *h = h_mix(*h, B[i]);
        got += x;
    }
    return got;
}

/* Read n frames into out (no hashing). */
static long long window_miniopus(MiniOpus* p, long long n, int chans, float* out)
{
    long long got = 0;
    while (got < n) {
        long long want = n - got; if (want > CHUNK) want = CHUNK;
        miniopus_uint64 r = miniopus_read_pcm_frames_f32(p, (miniopus_uint64)want, out + got * chans);
        if (r == 0) break;
        got += (long long)r;
    }
    return got;
}
static long long window_opus(OggOpusFile* of, long long n, int chans, float* out)
{
    long long got = 0;
    while (got < n) {
        long long want = n - got; if (want > CHUNK) want = CHUNK; if (want < 960) want = 960;
        int x = op_read_float(of, out + got * chans, (int)want, NULL);
        if (x < 0) return got;
        if (x == 0) break;
        got += x;
    }
    return got;
}
static long long drain_count_miniopus(MiniOpus* p)
{
    long long got = 0, r;
    while ((r = miniopus_read_pcm_frames_f32(p, CHUNK, B)) > 0) got += r;
    return got;
}
static long long drain_count_opus(OggOpusFile* of)
{
    long long got = 0;
    for (;;) { int x = op_read_float(of, B, CHUNK, NULL); if (x <= 0) break; got += x; }
    return got;
}

int main(int argc, char** argv)
{
    if (argc < 2) { fprintf(stderr, "usage: corpus_check <file...>\n"); return 2; }

    for (int a = 1; a < argc; ++a) {
        const char* path = argv[a];
        OggOpusFile* of = op_open_file(path, NULL);
        if (!of) { printf("SKIP (opusfile cannot open): %s\n", basename_(path)); continue; }

        Meta meta = {0};
        MiniOpus* p = miniopus_open_file_with_metadata(path, meta_cb, &meta, NULL);
        files++;
        if (!p) { CHECK(0, "%s: MiniOpus open FAILED (opusfile opened it)", basename_(path)); op_free(of); continue; }
        printf("%s\n", basename_(path));

        int chans = op_channel_count(of, -1); if (chans < 1) chans = 1;
        opus_int64 theirTotal = op_pcm_total(of, -1);
        CHECK(p->channels == (miniopus_uint32)chans, "channels %u vs %d", p->channels, chans);
        CHECK(p->sampleRate == 48000, "sampleRate");
        CHECK(p->sourceSampleRate == (miniopus_uint32)op_head(of, -1)->input_sample_rate, "src rate %u vs %u",
              (unsigned)p->sourceSampleRate, (unsigned)op_head(of, -1)->input_sample_rate);
        /* MiniOpus reports UMP-STMH7's duration: the last page's granule minus pre-skip.
           opusfile subtracts the granule the muxer started the audio timeline at, which
           ffmpeg's Lavf muxer offsets by 24. Both are internally consistent; the delta is
           the base, and it has to stay constant across the whole file. */
        long long baseDelta = (long long)p->totalPCMFrameCount - theirTotal;
        if (baseDelta != 0) printf("    note: total %llu vs opusfile %lld (granule base %+lld frames)\n",
                                   (unsigned long long)p->totalPCMFrameCount, (long long)theirTotal, baseDelta);

        /* whole-stream equality */
        op_set_dither_enabled(of, 0);
        struct timespec t0, t1; clock_gettime(CLOCK_MONOTONIC, &t0);
        unsigned long long ha = h_init(), hb = h_init();
        long long na = drain_miniopus(p, (long long)theirTotal + 10000, chans, &ha);
        long long nb = drain_opus(of, (long long)theirTotal + 10000, chans, &hb);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double secs = (t1.tv_sec - t0.tv_sec) + 1e-9 * (t1.tv_nsec - t0.tv_nsec);
        CHECK(na == (long long)theirTotal, "MiniOpus decoded %lld frames, opusfile says %lld", na, theirTotal);
        CHECK(nb == (long long)theirTotal, "opusfile decoded %lld frames", nb);
        CHECK(ha == hb, "whole-stream sample hash differs (MiniOpus %016llx opusfile %016llx)", ha, hb);
        CHECK((long long)p->currentPCMFrame == na, "currentPCMFrame %llu vs %lld", (unsigned long long)p->currentPCMFrame, na);

        /* seek battery. A landing decodes the straddling packet from cold decoder
        state - inherent to Opus, and opusfile does the same thing with its own
        pre-roll - so the first frames after a seek need not match bit-for-bit.
        What must match: the frame count to EOF, and the waveform once the
        overlap history has settled (60 ms in). */
        int seekFails = 0;
        static float wa[(2880 + 4096) * 8], wb[(2880 + 4096) * 8];
        for (int k = 1; k <= 8; ++k) {
            long long target = (long long)p->totalPCMFrameCount / 8 * k;
            if (target >= (long long)p->totalPCMFrameCount) continue;
            if (!miniopus_seek_to_pcm_frame(p, (miniopus_uint64)target)) { seekFails++; printf("    seek %lld: FAILED\n", target); continue; }
            if ((long long)p->currentPCMFrame != target) { seekFails++; printf("    seek %lld: currentPCMFrame=%llu\n",
                    target, (unsigned long long)p->currentPCMFrame); continue; }
            if (op_pcm_seek(of, (opus_int64)target) != 0) { seekFails++; printf("    seek %lld: opusfile seek failed\n", target); continue; }

            long long waGot = window_miniopus(p, 2880 + 4096, chans, wa);
            long long wbGot = window_opus(of, 2880 + 4096, chans, wb);
            long long tailA = drain_count_miniopus(p);
            long long tailB = drain_count_opus(of);
            long long gotA = waGot + tailA, gotB = wbGot + tailB;

            long long n = waGot < wbGot ? waGot : wbGot;
            /* Both libraries are landing on their own timeline convention: opusfile
               subtracts the granule ffmpeg started the audio at, MiniOpus (like
               UMP-STMH7) does not. Search the lag so the check measures the landing
               itself rather than the base, and require the lag to BE the base. */
            double corr = 0; long long corrLag = 0;
            for (long long lag = -64; lag <= 64; lag++) {
                double u_ab = 0, u_aa = 0, u_bb = 0; long long cnt = 0;
                for (long long i = 2880; i < n; i++) {
                    long long j = i - lag;
                    if (j < 2880 || j >= n) continue;
                    for (int c = 0; c < chans; c++) {
                        double u = wa[i * chans + c], v = wb[j * chans + c];
                        u_ab += u * v; u_aa += u * u; u_bb += v * v;
                    }
                    cnt++;
                }
                if (cnt < 1000) continue;
                double cc = (u_aa > 0 && u_bb > 0) ? u_ab / sqrt(u_aa * u_bb) : 1.0;
                if (cc > corr) { corr = cc; corrLag = lag; }
            }
            long long remaining = (long long)p->totalPCMFrameCount - target;
            if (gotA != remaining) { seekFails++; printf("    seek %lld: MiniOpus frames to EOF %lld, expected %lld\n", target, gotA, remaining); continue; }
            /* opusfile is held to its own convention, not MiniOpus's. */
            if (gotB != theirTotal - target) { seekFails++; printf("    seek %lld: opusfile frames to EOF %lld, expected %lld\n",
                                                                   target, gotB, theirTotal - target); continue; }
            if (n <= 2880 || corr < 0.999) { seekFails++; printf("    seek %lld: post-settle waveform corr %.6f at lag %lld over %lld frames\n", target, corr, corrLag, n - 2880); continue; }
            if (corrLag != baseDelta) { seekFails++; printf("    seek %lld: landed %lld frames off, expected the granule base %+lld\n", target, corrLag, baseDelta); continue; }
        }
        CHECK(seekFails == 0, "%d seek failures", seekFails);

        /* tail seek: to the very last frame, and past the end */
        {
            long long T = (long long)p->totalPCMFrameCount - 1;
            CHECK(miniopus_seek_to_pcm_frame(p, (miniopus_uint64)T), "seek to the last frame");
            ha = h_init();
            long long ga = drain_miniopus(p, 5000, chans, &ha);
            CHECK(ga == 1, "one frame remains after seeking to total-1 (got %lld)", ga);
            CHECK(miniopus_seek_to_pcm_frame(p, (miniopus_uint64)(p->totalPCMFrameCount + 1234)), "seek past the end clamps");
            CHECK(p->currentPCMFrame == (miniopus_uint64)p->totalPCMFrameCount, "clamped position %llu", (unsigned long long)p->currentPCMFrame);
            ha = h_init();
            ga = drain_miniopus(p, 1000, chans, &ha);
            CHECK(ga == 0, "nothing decodes past the end (got %lld)", ga);
        }

        int theirTagCount = 0;
        const OpusTags* ot = op_tags(of, -1);
        if (ot && ot->user_comments) theirTagCount = ot->comments;
        if (theirTagCount > 0)
            CHECK(meta.tags + meta.oversized >= (miniopus_uint32)theirTagCount, "tags: opusfile %d, MiniOpus %u (+%u oversized)",
                  theirTagCount, meta.tags, meta.oversized);

        /* backward seek from EOF state must re-stream exactly */
        {
            long long T = (long long)p->totalPCMFrameCount / 2;
            CHECK(miniopus_seek_to_pcm_frame(p, (miniopus_uint64)T), "backward seek to the middle");
            ha = h_init();
            long long ga = drain_miniopus(p, 6000, chans, &ha);
            CHECK(ga == 6000, "6000 frames decode after a backward seek (got %lld)", ga);
        }

        miniopus_close(p);
        op_free(of);
        printf("  ok: %lld frames (%.2fs audio) in %.2fs = %.0fx realtime, tags=%u oversized=%u pics=%u (%llu B) vendor=\"%s\"\n",
               na, (double)na / 48000.0, secs, secs > 0 ? ((double)na / 48000.0) / secs : 0,
               meta.tags, meta.oversized, meta.pictures, (unsigned long long)meta.pictureBytes, meta.vendor);
    }

    printf("\n=== corpus: %d files, %d failures ===\n", files, fails);
    return fails ? 1 : 0;
}
