#!/usr/bin/env bash
# Stitch recorded runs into one short demo video for the README:
#   title card -> external view of a landing -> onboard view of the same
#   landing -> inspection mission (onboard), each with a caption.
# Uses ffmpeg from the project image; no simulator needed.
#
# usage (repo root):
#   tools/make_demo_video.sh <observer.mp4> <onboard.mp4> <mission.mp4> <out.mp4> "<result line>"
# e.g.
#   tools/make_demo_video.sh output/demo/observer.mp4 output/demo/onboard.mp4 \
#       output/mission_3/mission.mp4 docs/media/demo.mp4 "This landing 3.2 cm from the pad centre"
set -euo pipefail
obs=$1; onb=$2; mis=$3; out=$4; result=${5:-}

FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
BOLD=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
tmp=$(mktemp -d -p "$PWD" .demo-XXXX)
trap 'rm -rf "$tmp"' EXIT

# Captions go through textfile= so no filtergraph escaping is needed.
printf '%s' "PX4 vision-based precision landing"                   > "$tmp/t1.txt"
printf '%s' "PX4 SITL + Gazebo Harmonic + OpenCV ArUco + MAVSDK"   > "$tmp/t2.txt"
printf '%s' "30 of 30 test landings by vision. Mean error 2.8 cm" > "$tmp/t3.txt"
printf '%s' "External view (2x speed)"                            > "$tmp/c1.txt"
printf '%s' "Same flight, onboard camera (1.5x). Marker hidden 3 s on purpose" > "$tmp/c2.txt"
printf '%s' "$result"                                             > "$tmp/c2b.txt"
printf '%s' "Inspection mission, onboard camera (6x speed)"        > "$tmp/c3.txt"

cap() {  # caption bar at the bottom: $1 = text file, optional $2 = second line file
  local f="drawbox=x=0:y=ih-56:w=iw:h=56:color=black@0.6:t=fill,drawtext=fontfile=$FONT:textfile=/w/$1:fontcolor=white:fontsize=18:x=12:y=h-46"
  [ -n "${2:-}" ] && f="$f,drawtext=fontfile=$FONT:textfile=/w/$2:fontcolor=0xffd84d:fontsize=16:x=12:y=h-22"
  printf '%s' "$f"
}
norm="scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2,fps=15,format=yuv420p,setsar=1"

cat > "$tmp/graph.txt" <<EOF
color=c=0x1b1f24:s=640x480:d=3.5,fps=15,format=yuv420p,setsar=1,
 drawtext=fontfile=$BOLD:textfile=/w/t1.txt:fontcolor=white:fontsize=26:x=(w-tw)/2:y=170,
 drawtext=fontfile=$FONT:textfile=/w/t2.txt:fontcolor=0xc8ccd2:fontsize=16:x=(w-tw)/2:y=220,
 drawtext=fontfile=$FONT:textfile=/w/t3.txt:fontcolor=0xffd84d:fontsize=18:x=(w-tw)/2:y=270[title];
[0:v]setpts=PTS/2,$norm,$(cap c1.txt)[a];
[1:v]setpts=PTS/1.5,$norm,$(cap c2.txt c2b.txt)[b];
[2:v]setpts=PTS/6,$norm,$(cap c3.txt)[c];
[title][a][b][c]concat=n=4:v=1:a=0[v]
EOF

docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/workspace -v "$tmp":/w -w /workspace px4-vision-landing:dev \
  ffmpeg -loglevel error -y -i "$obs" -i "$onb" -i "$mis" -filter_complex_script /w/graph.txt \
  -map "[v]" -c:v libx264 -preset slow -crf 27 -movflags +faststart "$out"
ls -la "$out"
