#!/usr/bin/env bash
# Turn a recorded run video into a README-sized GIF, using ffmpeg from the
# project image (no running simulator needed).
# usage (repo root): tools/make_gif.sh <in.mp4> <out.gif> [speedup=2] [width=480] [start_s=0] [duration_s]
set -euo pipefail
in=$1; out=$2; speed=${3:-2}; width=${4:-480}; start=${5:-0}; dur=${6:-}
trim="-ss $start"; [ -n "$dur" ] && trim="$trim -t $dur"
filters="setpts=PTS/${speed},fps=10,scale=${width}:-1:flags=lanczos"
docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/workspace -w /workspace px4-vision-landing:dev \
  ffmpeg -loglevel error -y $trim -i "$in" \
  -vf "${filters},split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" "$out"
ls -la "$out"
