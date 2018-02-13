#!/bin/bash
## This script creates the desired experiment folder structure for input and output files,
## as well as copies relavent xacro files for future reference.
## Laughlin Barker | JHU DSCL | July 2017

sim_name=$1

save_path="/home/laughlin/Documents/JHU/projects/uiNav/sim_data"
full_path="$save_path/$sim_name"

ice_model_path="/home/laughlin/ros_ws/src/under_ice_sim/models/ice"
ice_xacro_file="$ice_model_path/ice.xacro"
ice_gaz_file="$ice_model_path/ice.gazebo"

veh_model_path="/home/laughlin/ros_ws/src/uuv_simulator/uuv_descriptions/models/rexrov/urdf"
veh_xacro_file="$veh_model_path/rexrov_base_ui.xacro"
veh_gaz_file="$veh_model_path/rexrov_ui.gazebo.xacro"

cd "$save_path"

# export EXPERIMENT_NAME="$sim_name"

#check if experiment folder exists, if not create and populate
if [ ! -d "$sim_name" ]; then
	echo "Creating Gazebo output folder..."
	mkdir "$sim_name"

	cd "$sim_name"
	mkdir sim_models

	echo "Copying ice and vehicle models for your records..."
	cd sim_models
	cp "$ice_xacro_file" .
	cp "$ice_gaz_file" .
	cp "$veh_xacro_file" .
	cp "$veh_gaz_file" .

	echo "Creating bag and ekf_out folders..."
	cd "$full_path"
	mkdir bags
	mkdir figs

else

	echo "WARNING - Experiment folder already exists! Should I overwrite? [Y/N]: "
# 	read ow
# 	if [ "$ow" == "Y" ]; then
		echo "Overwriting folder contents..."

		##remove old data
		cd "$sim_name"
		rm -rf sim_models
		rm -rf bags

		mkdir sim_models

		echo "Copying ice and vehicle models for your records..."
		cd sim_models
		cp "$ice_xacro_file" .
		cp "$ice_gaz_file" .
		cp "$veh_xacro_file" .
		cp "$veh_gaz_file" .

		echo "Creating bag and ekf_out folders..."
		cd "$full_path"
		mkdir bags
# 	else
# 		echo "Leaving folders intact. Quitting..."
# 		exit 1
# 	fi

fi

