#!/usr/bin/env bash
# Record the SmolVLA policy driving the Viscous arm as a LeRobot dataset
# (camera videos + joint positions, one episode per run), using lerobot's
# episodic rollout strategy. Stop the Lab's inference session first: the
# CAN bus and cameras can only be open in one process.
#
# Keys while recording:  → end episode early   ← discard & redo   Esc stop
#
#   scripts/record-rollouts.sh [episodes=10] [seconds_per_episode=60] [reset_seconds=20]
set -euo pipefail
cd "$(dirname "$0")/.."
EPISODES="${1:-10}"
EPISODE_S="${2:-60}"
RESET_S="${3:-20}"
CKPT="$HOME/.cache/huggingface/lerobot/outputs/train/smolvla_gavingavinchan_white_cube_in_black_square_2_20260916_225331_2026-09-17_08-24-40/run/checkpoints/012000/pretrained_model"
PORT="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_205038953846-if00"
TASK="${TASK:-pick up the white cube and place into black square}"

exec .venv/bin/python -m makermodslab.maker_rollout \
  --strategy.type=episodic \
  --strategy.reset_to_initial_position=true \
  --inference.type=rtc \
  --policy.path="$CKPT" \
  --policy.device=cuda \
  --robot.type=viscous_follower \
  --robot.port="$PORT" \
  --robot.id=viscous_01 \
  --robot.cameras="{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: MJPG}, front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: MJPG} }" \
  --task="$TASK" \
  --fps=30 \
  --dataset.repo_id=gavingavinchan/rollout_white_cube_smolvla12k \
  --dataset.single_task="$TASK" \
  --dataset.fps=30 \
  --dataset.num_episodes="$EPISODES" \
  --dataset.episode_time_s="$EPISODE_S" \
  --dataset.reset_time_s="$RESET_S" \
  --dataset.streaming_encoding=true \
  --dataset.push_to_hub=false \
  --play_sounds=false \
  --return_to_initial_position=true
