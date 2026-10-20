/*
MiniOpus example: decode a file and report what the container says about itself.

    cc -std=c11 -O2 -DMINIOPUS_IMPLEMENTATION -I.. miniopus_decode_info.c ../miniopus.c \
       -o build/decode_info $(pkg-config --cflags --libs opus) -lm
    build/decode_info track.ogg 30

Arg 2 is an optional seek target in seconds. Prints the header map MiniOpus built
at open, the Vorbis comments, an embedded picture's header (without reading its
bytes when it is too big to buffer), and the cost of a seek in reads/seeks.

Same license as the library: Apache-2.0.
*/
#include "miniopus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void on_metadata(void* pUser, const miniopus_metadata* pMD)
{
    (void)pUser;
    if (pMD->type == MINIOPUS_METADATA_TAGS) {
        const miniopus_tags* t = &pMD->data.tags;
        printf("vendor: %s\n", t->vendorString);
        for (miniopus_uint32 i = 0; i < t->tagCount; i++)
            printf("  %s = %s\n", t->pTags[i].key, t->pTags[i].value);
        if (t->oversizedCommentCount)
            printf("  (%u comment(s) too large to buffer - counted, not delivered)\n",
                   t->oversizedCommentCount);
    } else {
        const miniopus_picture* p = &pMD->data.picture;
        printf("picture: type=%d %s %ux%u %u-bit  %llu bytes%s\n",
               p->pictureType, p->mimeType, p->width, p->height, p->depth,
               (unsigned long long)p->dataSize,
               p->pData ? "" : "  (header only - the bytes were streamed past)");
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s file.ogg [seek-seconds]\n", argv[0]); return 1; }

    MiniOpus* pOpus = miniopus_open_file_with_metadata(argv[1], on_metadata, NULL, NULL);
    if (!pOpus) {
        fprintf(stderr, "could not open %s\n", argv[1]);
        return 1;
    }

    printf("channels=%u  sampleRate=%u  sourceSampleRate=%u\n",
           pOpus->channels, pOpus->sampleRate, pOpus->sourceSampleRate);
    printf("duration=%.3f s  (%llu frames, seekable=%s)\n",
           (double)pOpus->totalPCMFrameCount / pOpus->sampleRate,
           (unsigned long long)pOpus->totalPCMFrameCount,
           miniopus_is_seekable(pOpus) ? "yes" : "no");

    if (argc > 2) {
        miniopus_uint64 target = (miniopus_uint64)(atof(argv[2]) * pOpus->sampleRate);
        if (!miniopus_seek_to_pcm_frame(pOpus, target)) {
            fprintf(stderr, "seek to %llu failed: result=%d ogg=%d\n",
                    (unsigned long long)target, (int)miniopus_last_error(pOpus), miniopus_last_ogg_error(pOpus));
            miniopus_close(pOpus);
            return 1;
        }
        printf("seeked to %.3f s (currentPCMFrame=%llu)\n",
               (double)pOpus->currentPCMFrame / pOpus->sampleRate,
               (unsigned long long)pOpus->currentPCMFrame);
    }

    char title[256];
    if (miniopus_find_tag(pOpus, "title", title, sizeof title)) printf("TITLE (via find_tag): %s\n", title);

    /* 200 ms of audio, in the format your output wants. */
    const miniopus_uint64 frames = pOpus->sampleRate / 5;
    miniopus_int16* pcm = (miniopus_int16*)malloc((size_t)frames * pOpus->channels * sizeof(miniopus_int16));
    const miniopus_uint64 got = miniopus_read_pcm_frames_s16(pOpus, frames, pcm);
    printf("read %llu frames of %d-bit PCM\n", (unsigned long long)got, 16);

    long peak = 0;
    for (miniopus_uint64 i = 0; i < got * pOpus->channels; i++) {
        long v = pcm[i] < 0 ? -pcm[i] : pcm[i];
        if (v > peak) peak = v;
    }
    printf("peak sample: %ld / 32767\n", peak);

    free(pcm);
    miniopus_close(pOpus);
    return 0;
}
