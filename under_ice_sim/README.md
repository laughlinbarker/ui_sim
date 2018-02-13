sample command list to run simulation:

roslaunch under_ice_sim empty_underwater_world.launch timeout:=1000
roslaunch uuv_descriptions upload_rexrov_ui.launch x:=1050 y:=0 z:=-30 yaw:=3.14
roslaunch under_ice_sim ice.launch folder:=test1
roslaunch under_ice_sim vehicle_pid_controller.launch
roslaunch under_ice_sim send_waypoints_lawnmower_1000x100_6_10.launch

gz world -p 0

Run EKF:
roslaunch ui_ekf defaultEKF.launch sim_folder:=test1 output_folder:=exp1
