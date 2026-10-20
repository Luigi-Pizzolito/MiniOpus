#!/bin/sh
# MiniOpus host test build/run. Needs libopus (pkg-config opus).
#
#   sh tests/build_and_run.sh
#
# Cross-checks against opusfile are compiled and run automatically when
# pkg-config finds it. To also cross-check against real .ogg files, point
# OGGTEST_DIR at a directory containing them:
#
#   OGGTEST_DIR=~/Music/opus sh tests/build_and_run.sh
set -e
cd "$(dirname "$0")"
CC=${CC:-cc}
OPUS_CFLAGS=$(pkg-config --cflags opus)
OPUS_LIBS=$(pkg-config --libs opus)
EXTRA_CFLAGS=""; EXTRA_LIBS=""
if pkg-config --exists opusfile 2>/dev/null; then
    EXTRA_CFLAGS="-DHAVE_OPUSFILE $(pkg-config --cflags opusfile)"
    EXTRA_LIBS="$(pkg-config --libs opusfile)"
fi
mkdir -p build

echo "== unit suite (125 checks: roundtrip, seek battery, metadata, broadcast, fuzz, opusfile cross, profiling)"
$CC -std=c11 -g -O2 -Wall -Wextra -Wno-unused-function \
    -DMINIOPUS_IMPLEMENTATION -I.. $OPUS_CFLAGS $EXTRA_CFLAGS \
    ../miniopus.c miniopus_tests.c -o build/miniopus_tests $OPUS_LIBS $EXTRA_LIBS -lm
build/miniopus_tests

echo
echo "== compile matrix (every configuration define, and the strict warning set)"
for cfg in "" \
           -DMINIOPUS_NO_FILEIO \
           -DMINIOPUS_NO_SEEKING \
           -DMINIOPUS_NO_METADATA \
           -DMINIOPUS_NO_ENCODER \
           -DMINIOPUS_CHECK_CRC \
           "-DMINIOPUS_NO_FILEIO -DMINIOPUS_NO_SEEKING -DMINIOPUS_NO_METADATA -DMINIOPUS_NO_ENCODER" \
           "-DMINIOPUS_NO_FILEIO -DMINIOPUS_NO_STDIO_H"
do
    printf '   %-78s ' "${cfg:-(default)}"
    $CC -std=c11 -O2 -Wall -Wextra -Wshadow -Wconversion -Wno-unused-function \
        -DMINIOPUS_IMPLEMENTATION $cfg -I.. ../miniopus.c -c -o build/matrix.o $OPUS_CFLAGS
    echo "clean"
done
printf '   %-78s ' "(header alone, C99 and C++)"
$CC  -std=c99 -Wall -Wextra -I.. -c ../miniopus.h -o build/hdr_c99.o
$CC -x c++ -std=c++17 -Wall -Wextra -I.. -c ../miniopus.h -o build/hdr_cpp.o
echo "clean"

echo
echo "== examples (built and run against a file the encoder example writes)"
$CC -std=c11 -O2 -Wall -Wextra -DMINIOPUS_IMPLEMENTATION -I.. \
    ../examples/miniopus_encode_file.c ../miniopus.c -o build/encode_example $OPUS_CFLAGS $OPUS_LIBS -lm
$CC -std=c11 -O2 -Wall -Wextra -DMINIOPUS_IMPLEMENTATION -I.. \
    ../examples/miniopus_decode_info.c ../miniopus.c -o build/decode_example $OPUS_CFLAGS $OPUS_LIBS -lm
build/encode_example build/miniopus_example.ogg
build/decode_example build/miniopus_example.ogg 1.0 | tail -4

if [ -n "$OGGTEST_DIR" ]; then
    if [ -z "$EXTRA_LIBS" ]; then
        echo
        echo "== corpus cross-check skipped: opusfile not found by pkg-config"
    else
        echo
        echo "== corpus cross-check against $OGGTEST_DIR"
        $CC -std=c11 -O2 -Wall -Wextra -Wno-unused-function \
            -DHAVE_OPUSFILE -DMINIOPUS_IMPLEMENTATION -I.. $OPUS_CFLAGS $EXTRA_CFLAGS \
            corpus_check.c ../miniopus.c -o build/corpus_check $OPUS_LIBS $EXTRA_LIBS -lm
        build/corpus_check "$OGGTEST_DIR"/*.ogg
    fi
fi
