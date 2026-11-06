#!/usr/bin/env bash
set -euo pipefail

# install_voxblox_deps_no_rosdep.sh
#
# Purpose:
#   Install and build Voxblox + common ETH ASL catkin dependencies for ROS1 Noetic
#   without relying on rosdep.
#
# Tested scenario:
#   Ubuntu 20.04 + ROS Noetic + catkin_tools
#
# Usage:
#   chmod +x install_voxblox_deps_no_rosdep.sh
#   ./install_voxblox_deps_no_rosdep.sh
#
# Optional:
#   WORKSPACE=/home/patrick/sim_dev/simulation_framework ./install_voxblox_deps_no_rosdep.sh
#   ROS_DISTRO=noetic ./install_voxblox_deps_no_rosdep.sh
#
# Notes:
#   - This script does not modify your existing project code.
#   - It only clones missing dependency repositories into ${WORKSPACE}/src
#     and builds the required catkin packages.
#   - If a repository directory already exists, it will be skipped.
#   - It does not run rosdep update/install.

WORKSPACE="${WORKSPACE:-$HOME/sim_dev/simulation_framework}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SRC_DIR="${WORKSPACE}/src"

echo "[INFO] Workspace: ${WORKSPACE}"
echo "[INFO] ROS distro: ${ROS_DISTRO}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "[ERROR] /opt/ros/${ROS_DISTRO}/setup.bash not found."
  echo "        Please install ROS ${ROS_DISTRO} first, or set ROS_DISTRO correctly."
  exit 1
fi

if ! command -v catkin >/dev/null 2>&1; then
  echo "[ERROR] catkin command not found."
  echo "        Please install catkin_tools first:"
  echo "        sudo apt install python3-catkin-tools"
  exit 1
fi

mkdir -p "${SRC_DIR}"

echo "[INFO] Installing minimal system dependencies via apt..."
sudo apt update
sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-catkin-tools \
  libeigen3-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libceres-dev

clone_if_missing() {
  local repo_url="$1"
  local target_dir="$2"

  if [ -d "${SRC_DIR}/${target_dir}" ]; then
    echo "[SKIP] ${target_dir} already exists."
  else
    echo "[CLONE] ${repo_url} -> ${SRC_DIR}/${target_dir}"
    git clone "${repo_url}" "${SRC_DIR}/${target_dir}"
  fi
}

echo "[INFO] Cloning catkin / ETH ASL dependencies if missing..."

clone_if_missing "https://github.com/catkin/catkin_simple.git" "catkin_simple"

clone_if_missing "https://github.com/ethz-asl/eigen_catkin.git" "eigen_catkin"
clone_if_missing "https://github.com/ethz-asl/eigen_checks.git" "eigen_checks"
clone_if_missing "https://github.com/ethz-asl/glog_catkin.git" "glog_catkin"
clone_if_missing "https://github.com/ethz-asl/gflags_catkin.git" "gflags_catkin"
clone_if_missing "https://github.com/ethz-asl/protobuf_catkin.git" "protobuf_catkin"

clone_if_missing "https://github.com/ethz-asl/minkindr.git" "minkindr"
clone_if_missing "https://github.com/ethz-asl/minkindr_ros.git" "minkindr_ros"

clone_if_missing "https://github.com/ethz-asl/voxblox.git" "voxblox"

cd "${WORKSPACE}"

echo "[INFO] Sourcing ROS..."
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "[INFO] Configuring catkin workspace..."
catkin config --extend "/opt/ros/${ROS_DISTRO}" --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[INFO] Building dependency chain..."

catkin build catkin_simple
# shellcheck disable=SC1090
source "${WORKSPACE}/devel/setup.bash"

catkin build \
  eigen_catkin \
  glog_catkin \
  gflags_catkin \
  eigen_checks \
  protobuf_catkin \
  minkindr \
  minkindr_conversions

# shellcheck disable=SC1090
source "${WORKSPACE}/devel/setup.bash"

echo "[INFO] Building Voxblox..."
catkin build voxblox_msgs voxblox voxblox_ros

# shellcheck disable=SC1090
source "${WORKSPACE}/devel/setup.bash"

echo "[INFO] Verifying packages..."
rospack find catkin_simple
rospack find eigen_catkin
rospack find eigen_checks
rospack find glog_catkin
rospack find gflags_catkin
rospack find protobuf_catkin
rospack find minkindr
rospack find voxblox
rospack find voxblox_msgs
rospack find voxblox_ros

echo ""
echo "[SUCCESS] Voxblox dependency setup completed."
echo ""
echo "Next time, source your workspace before using Voxblox:"
echo "  source ${WORKSPACE}/devel/setup.bash"
echo ""
echo "For zsh users:"
echo "  source ${WORKSPACE}/devel/setup.zsh"
