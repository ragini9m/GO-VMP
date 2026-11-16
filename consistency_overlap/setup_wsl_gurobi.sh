#!/usr/bin/env bash
# Sets up Gurobi 11.0.1 (the version GO-VMP uses) and OctoMap in WSL Ubuntu,
# activates the Gurobi licence, and builds + runs claude_implementation.cpp.
#
# Usage (inside WSL):
#   bash consistency_overlap/setup_wsl_gurobi.sh <your-gurobi-licence-key>
#
# The key is only passed to grbgetkey; it is not written to any file by this script.
set -euo pipefail

KEY="${1:-}"
if [ -z "$KEY" ]; then
	echo "Usage: bash setup_wsl_gurobi.sh <gurobi-licence-key>"
	exit 1
fi

GRB_VERSION="11.0.1"        # matches GO-VMP/src/view_motion_planner/CMakeLists.txt
GRB_DIR="gurobi1101"
GRB_LIB="gurobi110"
THESIS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # folder of this script (consistency_overlap/)
export GUROBI_HOME="/opt/${GRB_DIR}/linux64"

echo "== 1/6 System packages (compiler, OctoMap) =="
sudo apt-get update
sudo apt-get install -y build-essential wget liboctomap-dev

echo "== 2/6 Download and unpack Gurobi ${GRB_VERSION} to /opt =="
if [ ! -d "$GUROBI_HOME" ]; then
	cd /tmp
	wget -q --show-progress "https://packages.gurobi.com/${GRB_VERSION%.*}/gurobi${GRB_VERSION}_linux64.tar.gz"
	sudo tar xzf "gurobi${GRB_VERSION}_linux64.tar.gz" -C /opt
	rm "gurobi${GRB_VERSION}_linux64.tar.gz"
else
	echo "already present: $GUROBI_HOME"
fi

echo "== 3/6 Environment variables in ~/.bashrc =="
if ! grep -q "GUROBI_HOME=" ~/.bashrc; then
	{
		echo ""
		echo "# Gurobi"
		echo "export GUROBI_HOME=\"$GUROBI_HOME\""
		echo 'export PATH="$GUROBI_HOME/bin:$PATH"'
		echo 'export LD_LIBRARY_PATH="$GUROBI_HOME/lib:${LD_LIBRARY_PATH:-}"'
	} >> ~/.bashrc
fi
export PATH="$GUROBI_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$GUROBI_HOME/lib:${LD_LIBRARY_PATH:-}"

echo "== 4/6 Rebuild the C++ wrapper library with this system's compiler =="
# The shipped libgurobi_c++.a can be ABI-incompatible with newer GCC versions.
cd "$GUROBI_HOME/src/build"
sudo make -s
sudo cp libgurobi_c++.a "$GUROBI_HOME/lib/"

echo "== 5/6 Activate the licence (grbgetkey) =="
# Academic licences are validated against the university network:
# run this while connected to the Uni Bonn network or VPN.
# grbgetkey asks where to save gurobi.lic; accept the default (your home directory).
if [ ! -f ~/gurobi.lic ]; then
	grbgetkey "$KEY"
else
	echo "~/gurobi.lic already exists, skipping grbgetkey"
fi
echo "-- licence test: solve Gurobi's example model --"
gurobi_cl "$GUROBI_HOME/examples/data/coins.lp" | tail -n 5

echo "== 6/6 Build and run claude_implementation.cpp =="
BUILD="$HOME/thesis_build"
mkdir -p "$BUILD"
g++ -std=c++14 -O2 "$THESIS/claude_implementation.cpp" -o "$BUILD/claude_implementation" \
	-I"$GUROBI_HOME/include" -L"$GUROBI_HOME/lib" -lgurobi_c++ -l"$GRB_LIB" -loctomap -loctomath
"$BUILD/claude_implementation"
