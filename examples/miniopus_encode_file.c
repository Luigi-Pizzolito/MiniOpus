/*
MiniOpus example: write an Ogg Opus file, then read it back.

    cc -std=c11 -O2 -DMINIOPUS_IMPLEMENTATION -I.. miniopus_encode_file.c ../miniopus.c \
       -o build/encode_file $(pkg-config --cflags --libs opus) -lm
    build/encode_file out.ogg

Synthesises two seconds of rising tone, encodes it with Vorbis comments attached,
reopens the result and reports what the container says - including whether the
duration the encoder wrote matches the frames the decoder reads back, which is the
one thing an Ogg Opus writer most easily gets wrong.

Same license as the library: Apache-2.0.
*/
#include "miniopus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

int main(int argc, char** argv)
{
    const char* path = (argc > 1) ? argv[1] : "miniopus_example.ogg";
    const miniopus_uint32 rate = 48000, channels = 2;
    const miniopus_uint64 frames = (miniopus_uint64)rate * 2;      /* two seconds */

    miniopus_int16* pcm = (miniopus_int16*)malloc((size_t)frames * channels * sizeof(miniopus_int16));
    if (!pcm) return 1;
    for (miniopus_uint64 i = 0; i < frames; i++) {
        double t = (double)i / rate;
        double f = 220.0 + 660.0 * ((double)i / (double)frames);           /* 220 -> 880 Hz */
        double env = 0.35 * (1.0 - fabs(fmod(t, 0.5) * 2.0));              /* a little pulse */
        double s = sin(2.0 * M_PI * f * t) * env;
        miniopus_int16 v = (miniopus_int16)(s * 20000.0);
        pcm[i * channels + 0] = v;
        pcm[i * channels + 1] = v;
    }

    miniopus_tag tags[] = {
        { "TITLE",       "Rising Tone" },
        { "ARTIST",      "MiniOpus example" },
        { "COMMENT",     "written by the MiniOpus muxer, seekable by granule bisection" }
    };
    miniopus_encoder_config cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.application    = MINIOPUS_APPLICATION_AUDIO;
    cfg.bitrateBps     = 128000;
    cfg.vbr            = 1;
    cfg.complexity     = 10;
    cfg.frameMsX10     = 200;                 /* 20 ms frames */
    cfg.pTags          = tags;
    cfg.tagCount       = (miniopus_uint32)(sizeof(tags) / sizeof(tags[0]));
    cfg.pVendorString  = "MiniOpus example";

    miniopus_result r = miniopus_encode_pcm16_to_opus_file(path, pcm, frames, rate, channels, &cfg, NULL);
    free(pcm);
    if (r != MINIOPUS_OK) { fprintf(stderr, "encode failed: %d\n", (int)r); return 1; }

    MiniOpus* pOpus = miniopus_open_file(path, NULL);
    if (!pOpus) { fprintf(stderr, "could not re-open %s\n", path); return 1; }

    printf("wrote %s: %llu frames encoded, decoder reports %llu (%.3f s), channels=%u\n",
           path, (unsigned long long)frames,
           (unsigned long long)pOpus->totalPCMFrameCount,
           (double)pOpus->totalPCMFrameCount / pOpus->sampleRate, pOpus->channels);

    char title[128];
    if (miniopus_find_tag(pOpus, "title", title, sizeof title)) printf("TITLE read back: %s\n", title);

    /* Seek to the middle and decode the rest, counting frames: an encoder whose
     * granules are wrong shows up here as a mismatch against the declared total. */
    const miniopus_uint64 mid = pOpus->totalPCMFrameCount / 2;
    if (!miniopus_seek_to_pcm_frame(pOpus, mid)) { fprintf(stderr, "seek failed\n"); miniopus_close(pOpus); return 1; }

    miniopus_uint64 got = 0, block;
    miniopus_int16 buf[2048 * 2];
    while ((block = miniopus_read_pcm_frames_s16(pOpus, 2048, buf)) > 0) got += block;

    printf("after seek to %llu: %llu frames decoded, expected %llu -> %s\n",
           (unsigned long long)mid, (unsigned long long)got,
           (unsigned long long)(pOpus->totalPCMFrameCount - mid),
           (got == pOpus->totalPCMFrameCount - mid) ? "exact" : "MISMATCH");

    miniopus_close(pOpus);
    return (got == pOpus->totalPCMFrameCount - mid) ? 0 : 1;
}
