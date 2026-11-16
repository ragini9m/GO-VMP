#!/usr/bin/env bash
# Runs a Gurobi program in WSL despite WSL's changing network address.
#
# Why: the Gurobi named-user licence is tied to a host ID that Gurobi derives
# from the network card address. WSL gives the card a new random address after
# every restart, so the licence stops matching. This script temporarily sets
# the address the licence was activated with, runs the program, and then
# restores the previous address. While the program runs, WSL has no internet;
# Gurobi does not need it.
#
# Usage (inside WSL, from any folder):
#   bash consistency_overlap/gurobi_run.sh ~/thesis_build/claude_implementation
# Asks for your Linux password (sudo) to change the network address.
set -u

# Address the licence was activated with: HOSTID in ~/gurobi.lic, e.g. HOSTID=5d50d6ca -> 00:15:5d:50:d6:ca
LICENSED_MAC="${GUROBI_LICENSED_MAC:-00:15:5d:50:d6:ca}"
DEV="eth0"

if [ $# -eq 0 ]; then
	echo "Usage: bash gurobi_run.sh <program> [arguments]"
	exit 1
fi

CURRENT_MAC=$(cat /sys/class/net/$DEV/address)
GATEWAY=$(ip route show default | awk '{print $3; exit}')

set_mac() {
	sudo ip link set dev "$DEV" down
	sudo ip link set dev "$DEV" address "$1"
	sudo ip link set dev "$DEV" up
	ip route show default | grep -q default || sudo ip route add default via "$GATEWAY" dev "$DEV"
}

restore() {
	if [ "$(cat /sys/class/net/$DEV/address)" != "$CURRENT_MAC" ]; then
		set_mac "$CURRENT_MAC"
		echo "[gurobi_run] network address restored, internet available again"
	fi
}

if [ "$CURRENT_MAC" != "$LICENSED_MAC" ]; then
	sudo -v || exit 1
	trap restore EXIT INT TERM
	set_mac "$LICENSED_MAC"
	echo "[gurobi_run] licensed address set (no internet while the program runs)"
fi

export GUROBI_HOME="${GUROBI_HOME:-/opt/gurobi1101/linux64}"
export LD_LIBRARY_PATH="$GUROBI_HOME/lib:${LD_LIBRARY_PATH:-}"
"$@"
