export WORKSPACE=${WORKSPACE:-/root/workspace}

# shellcheck disable=SC1090
source "$WORKSPACE"/setup-workspace.sh
roslaunch f1tenth_simulator simulator.launch map:="$WORKSPACE"/code/f1tenth-racing-algorithms/maps/vegas.yaml