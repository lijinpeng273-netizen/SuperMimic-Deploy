#!/bin/bash
# Build h1_bridge + libh1bridge_client.so:  bash build.sh <unitree_sdk2 dir>   (x86_64 deploy PC or the aarch64 onboard computer PC2)
set -e
SDK=${1:?unitree_sdk2 dir}
ARCH=$(uname -m)
cd "$(dirname "$0")"
TL=$SDK/thirdparty/lib/$ARCH
g++ -O2 -std=c++17 -Wall -Wno-unused-parameter -I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx" \
    h1_bridge.cpp -o h1_bridge -L"$SDK/lib/$ARCH" -lunitree_sdk2 -L"$TL" -lddscxx -lddsc -lpthread -lrt \
    -Wl,--disable-new-dtags,-rpath,"$TL"
g++ -O2 -std=c++17 -Wall -shared -fPIC h1_bridge_client.cpp -o libh1bridge_client.so -lrt
echo "built $(pwd)/h1_bridge $(pwd)/libh1bridge_client.so ($ARCH)"
