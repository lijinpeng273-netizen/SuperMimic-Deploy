#!/usr/bin/env bash
# Build only the read-only pose bridge; leave the motor bridge and its ABI alone.
set -euo pipefail
SDK=${1:?usage: bash build_pose.sh /path/to/unitree_sdk2}
ARCH=$(uname -m)
cd "$(dirname "$0")"
TL=$SDK/thirdparty/lib/$ARCH
g++ -O2 -std=c++17 -Wall -Wextra -Wno-unused-parameter \
  -I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx" \
  h1_odom_bridge.cpp -o h1_odom_bridge -L"$SDK/lib/$ARCH" -lunitree_sdk2 \
  -L"$TL" -lddscxx -lddsc -lpthread -lrt -Wl,--disable-new-dtags,-rpath,"$TL"
echo "built $(pwd)/h1_odom_bridge ($ARCH); no motor bridge changes"
