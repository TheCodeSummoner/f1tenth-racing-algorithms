export WORKSPACE=${WORKSPACE:-/root/workspace}
export SIMULATOR_RVIZ_FILE=${SIMULATOR_RVIZ_FILE:-$WORKSPACE/simulator/src/f1tenth_simulator/launch/simulator.rviz}

yq eval '.["Visualization Manager"].Displays += {"Class": "rviz/Marker", "Enabled": true, "Marker Topic": "/visualisation_marker_01", "Name": "Visualisation Marker 01", "Namespaces": {}, "Queue Size": 100, "Value": true}' "$SIMULATOR_RVIZ_FILE" --inplace
yq eval '.["Visualization Manager"].Displays += {"Class": "rviz/Marker", "Enabled": true, "Marker Topic": "/visualisation_marker_02", "Name": "Visualisation Marker 02", "Namespaces": {}, "Queue Size": 100, "Value": true}' "$SIMULATOR_RVIZ_FILE" --inplace
yq eval '.["Visualization Manager"].Displays += {"Class": "rviz/Marker", "Enabled": true, "Marker Topic": "/visualisation_marker_03", "Name": "Visualisation Marker 03", "Namespaces": {}, "Queue Size": 100, "Value": true}' "$SIMULATOR_RVIZ_FILE" --inplace
yq eval '.["Visualization Manager"].Displays += {"Class": "rviz/Marker", "Enabled": true, "Marker Topic": "/visualisation_marker_04", "Name": "Visualisation Marker 04", "Namespaces": {}, "Queue Size": 100, "Value": true}' "$SIMULATOR_RVIZ_FILE" --inplace
yq eval '.["Visualization Manager"].Displays += {"Class": "rviz/Marker", "Enabled": true, "Marker Topic": "/visualisation_marker_05", "Name": "Visualisation Marker 05", "Namespaces": {}, "Queue Size": 100, "Value": true}' "$SIMULATOR_RVIZ_FILE" --inplace
