#!/usr/bin/env bash
# Smoke test for the recorder's start gate and tf capture (format v4).
# NOT collected by pytest (not named test_*.py). Needs a ROS 2 install and takes ~40 s.
#
#   bash test/smoke_gate_tf.sh [OUT_DIR]        # all three cases
#   CASES="abort" bash test/smoke_gate_tf.sh    # a subset: wait abort allow_missing
#
# Runs the recorder from source (PYTHONPATH=.) on an isolated ROS graph
# (ROS_DOMAIN_ID=77, localhost only) against `ros2 topic pub` fakes:
#   /smoke/odom (Odometry, always), /smoke/cmd_vel (Twist, the "late" stream),
#   /tf (odom->base_link and odom->other), /tf_static (base_link->radar).
# Cases (PASS/FAIL printed for each, exit status 1 if any FAIL):
#   wait          cmd_vel starts 6 s late; the gate must wait, then open; the file has
#                 /tf_static base_link->radar, /tf/odom__base_link n>0, no odom__other
#   abort         cmd_vel never starts; exit code 2, log names the topic, no file
#   allow_missing same, with allow_missing: file exists, streams_missing == [cmd_vel]
# Logs, params and .h5 files are kept in OUT_DIR (default: a fresh temp dir).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # src/dataset_generator
[ -f /opt/ros/jazzy/setup.bash ] && ! command -v ros2 >/dev/null && source /opt/ros/jazzy/setup.bash
# the python used for the recorder and the h5 checks (needs h5py + the ROS python path)
PY="${PY:-uv run --project $HERE/../.. --frozen --with h5py python -u}"
OUT="${1:-$(mktemp -d -t gate_tf_smoke.XXXXXX)}"
mkdir -p "$OUT"
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$HERE:${PYTHONPATH:-}"
CASES="${CASES:-wait abort allow_missing}"
echo "smoke_gate_tf: output in $OUT"

PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -- "-$p" 2>/dev/null; done; PIDS=(); }
trap cleanup EXIT

bg() {   # start a fake publisher in its own process group
    setsid "$@" >>"$OUT/fakes.log" 2>&1 &
    PIDS+=("$!")
}

TF_MSG='{transforms: [
 {header: {frame_id: odom}, child_frame_id: base_link, transform: {translation: {x: 1.0}, rotation: {w: 1.0}}},
 {header: {frame_id: odom}, child_frame_id: other, transform: {translation: {x: 5.0}, rotation: {w: 1.0}}}]}'

start_fakes() {   # always-on: odom, tf, tf_static
    bg ros2 topic pub -r 10 /smoke/odom nav_msgs/msg/Odometry '{header: {frame_id: odom}}'
    bg ros2 topic pub -r 10 /tf tf2_msgs/msg/TFMessage "$TF_MSG"
    bg ros2 run tf2_ros static_transform_publisher --x 0.1 --z 0.2 --frame-id base_link --child-frame-id radar
}

write_params() {   # case timeout allow_missing duration
    cat >"$OUT/$1.yaml" <<YAML
dataset_generator:
  ros__parameters:
    output_file: "$OUT/$1.h5"
    start_timeout_s: $2
    allow_missing: $3
    duration_s: $4
    flush_period_s: 1.0
    tf_track: ["odom->base_link"]
    streams:
      odom:
        topic: "/smoke/odom"
        type: "nav_msgs/msg/Odometry"
      cmd_vel:
        topic: "/smoke/cmd_vel"
        type: "geometry_msgs/msg/Twist"
YAML
}

run_recorder() {   # case -> sets RC
    $PY -m dataset_generator.dataset_recorder --ros-args --params-file "$OUT/$1.yaml" \
        >"$OUT/$1.log" 2>&1
    RC=$?
}

FAILED=0
verdict() {   # case ok(0/1) message
    if [ "$2" = 0 ]; then echo "PASS $1"; else echo "FAIL $1: $3"; FAILED=1; fi
}

case_wait() {
    write_params wait 30.0 false 4.0
    start_fakes
    bg bash -c 'sleep 6; date +%s%N >"$1/late_start_ns"; exec ros2 topic pub -r 10 /smoke/cmd_vel geometry_msgs/msg/Twist "{}"' _ "$OUT"
    run_recorder wait
    cleanup
    local why=""
    [ "$RC" = 0 ] || why="exit code $RC (want 0)"
    grep -q 'GATE waiting missing=.*/smoke/cmd_vel' "$OUT/wait.log" || why="$why; no 'GATE waiting' line naming /smoke/cmd_vel"
    grep -q 'GATE open' "$OUT/wait.log" || why="$why; no 'GATE open' line"
    [ -f "$OUT/wait.h5" ] || why="$why; no file"
    if [ -f "$OUT/wait.h5" ]; then
        local out
        out="$($PY - "$OUT/wait.h5" "$(cat "$OUT/late_start_ns" 2>/dev/null || echo 0)" <<'PYEOF'
import sys
import h5py
f = h5py.File(sys.argv[1], 'r')
late = int(sys.argv[2])
bad = []
if not f.attrs['closed_cleanly']:
    bad.append('not closed cleanly')
if int(f.attrs['format_version']) != 4:
    bad.append('format_version %s' % f.attrs['format_version'])
if int(f.attrs['start_recv_ns']) < late:
    bad.append('start_recv_ns %d before late publisher start %d' % (f.attrs['start_recv_ns'], late))
if list(f.attrs['streams_missing']):
    bad.append('streams_missing %s' % list(f.attrs['streams_missing']))
g = f['tf_static']
pairs = list(zip(g['parent'].asstr()[:], g['child'].asstr()[:]))
if ('base_link', 'radar') not in pairs:
    bad.append('tf_static has %s' % pairs)
if 'odom__base_link' not in f.get('tf', {}) or f['tf/odom__base_link'].attrs['n_samples'] <= 0:
    bad.append('no /tf/odom__base_link samples')
if 'odom__other' in f.get('tf', {}):
    bad.append('untracked odom__other group present')
for s in ('odom', 'cmd_vel'):
    if f[s].attrs['n_samples'] <= 0:
        bad.append('%s has no samples' % s)
print('; '.join(bad))
PYEOF
)"
        [ -z "$out" ] || why="$why; $out"
    fi
    why="${why#; }"
    if [ -z "$why" ]; then verdict wait 0; else verdict wait 1 "$why"; fi
}

case_abort() {
    write_params abort 5.0 false 0.0
    start_fakes
    run_recorder abort
    cleanup
    local why=""
    [ "$RC" = 2 ] || why="exit code $RC (want 2)"
    grep -q 'GATE abort missing=.*/smoke/cmd_vel' "$OUT/abort.log" || why="$why; no 'GATE abort' line naming /smoke/cmd_vel"
    [ ! -e "$OUT/abort.h5" ] || why="$why; a file exists"
    why="${why#; }"
    if [ -z "$why" ]; then verdict abort 0; else verdict abort 1 "$why"; fi
}

case_allow_missing() {
    write_params allow_missing 5.0 true 3.0
    start_fakes
    run_recorder allow_missing
    cleanup
    local why=""
    [ "$RC" = 0 ] || why="exit code $RC (want 0)"
    if [ -f "$OUT/allow_missing.h5" ]; then
        local out
        out="$($PY - "$OUT/allow_missing.h5" <<'PYEOF'
import sys
import h5py
f = h5py.File(sys.argv[1], 'r')
miss = list(f.attrs['streams_missing'])
print('' if miss == ['cmd_vel'] else 'streams_missing %s (want [cmd_vel])' % miss)
PYEOF
)"
        [ -z "$out" ] || why="$why; $out"
    else
        why="$why; no file"
    fi
    why="${why#; }"
    if [ -z "$why" ]; then verdict allow_missing 0; else verdict allow_missing 1 "$why"; fi
}

for c in $CASES; do
    cleanup
    "case_$c"
done
cleanup
[ "$FAILED" = 0 ] && echo "smoke_gate_tf: ALL PASS" || echo "smoke_gate_tf: FAILURES (see $OUT)"
exit "$FAILED"
