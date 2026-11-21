#!/usr/bin/env bash
# Run a command with this workspace and private copies of Noetic's OpenSSL libs.
# The copies avoid an observed kernel mmap/unmap stall on the shared system files.
set -eo pipefail

if [ "$#" -eq 0 ]; then
  echo "Usage: bash tools/with_noetic_runtime.sh COMMAND [ARG ...]" >&2
  exit 2
fi

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_setup="$repo_root/swarm-playground/formation_ws/devel/original_baseline/setup.bash"
if [ ! -r /opt/ros/noetic/setup.bash ] || [ ! -r "$workspace_setup" ]; then
  echo "Run this script inside the Noetic container with original_baseline built." >&2
  exit 1
fi

runtime_dir="$(mktemp -d /tmp/ego-noetic-runtime.XXXXXX)"
trap 'rm -rf -- "$runtime_dir"' EXIT
cp /usr/lib/x86_64-linux-gnu/libssl.so.1.1 \
   /usr/lib/x86_64-linux-gnu/libcrypto.so.1.1 "$runtime_dir/"
export LD_LIBRARY_PATH="$runtime_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

source /opt/ros/noetic/setup.bash
source "$workspace_setup"
export LD_LIBRARY_PATH="$runtime_dir:$LD_LIBRARY_PATH"
"$@"
