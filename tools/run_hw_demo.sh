#!/bin/bash
# Run one warehouse scenario with Webots driving the REAL Nexys 3 over Ethernet and make all the media.
#   tools/run_hw_demo.sh SEED [TARGET]     TARGET defaults to 192.168.10.255 (the board); use 127.0.0.1 with bridge/mock_fpga_ff.py
# Needs: the board programmed (fpga/program.sh), Webots nightly, ffmpeg, matplotlib. Outputs:
#   sim/ff2_webots_hw_s<SEED>.npz   recording of the run (cams, pose, wheels, masks)
#   media/robot_eye_hw_seed<SEED>.mp4, media/webots_hw_seed<SEED>_3d.mp4, media/topdown_hw_seed<SEED>.mp4
# Do not pass --minimize to Webots (it stalls). Two runs are needed: the colour eye camera adds a big overlay to
# Webots' own 3D movie, so the movie is recorded in a second, identical run.
set -e
cd "$(dirname "$0")/.."
SEED=${1:?seed}; TARGET=${2:-192.168.10.255}
WEBOTS=/Applications/Webots.app/Contents/MacOS/webots
run() {  # $1 = log, rest = env assignments handled by caller
  PYTHONUNBUFFERED=1 HIL_BRAIN=udp HIL_TARGET=$TARGET "$@"
}
rm -rf sim/eye_hw$SEED
python3 webots/make_world.py --seed $SEED --quit-at-goal -o webots/worlds/warehouse_test.wbt
HIL_RECORD=$PWD/sim/ff2_webots_hw_s$SEED.npz HIL_EYE_DIR=$PWD/sim/eye_hw$SEED \
  PYTHONUNBUFFERED=1 HIL_BRAIN=udp HIL_TARGET=$TARGET $WEBOTS --batch --mode=fast --stdout --stderr \
  webots/worlds/warehouse_test.wbt > sim/webots2_hw_s$SEED.log 2>&1
grep -E "box dropped|at_goal|timeouts|stopped|limit" sim/webots2_hw_s$SEED.log
python3 ref/check_ff_run.py sim/ff2_webots_hw_s$SEED.npz --seed $SEED | tail -7 | tee sim/check2_hw_s$SEED.txt
ffmpeg -loglevel error -y -framerate 31 -i sim/eye_hw$SEED/f%05d.png -c:v libx264 -pix_fmt yuv420p -crf 22 media/robot_eye_hw_seed$SEED.mp4
python3 webots/make_world.py --seed $SEED --quit-at-goal --movie media/webots_hw_seed${SEED}_3d.mp4 -o webots/worlds/warehouse_test.wbt > /dev/null
HIL_RECORD=$PWD/sim/ff2_webots_hw_s${SEED}b.npz PYTHONUNBUFFERED=1 HIL_BRAIN=udp HIL_TARGET=$TARGET $WEBOTS --batch --mode=fast \
  --stdout --stderr webots/worlds/warehouse_test.wbt > sim/webots2_hw_s${SEED}b.log 2>&1
grep -E "movie saved|timeouts" sim/webots2_hw_s${SEED}b.log
python3 ref/render_video.py sim/ff2_webots_hw_s$SEED.npz --seed $SEED -o media/topdown_hw_seed$SEED.mp4 \
  --title "Real Nexys 3 FPGA driving the Webots warehouse robot (seed $SEED)" | tail -1
