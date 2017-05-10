#!/bin/bash

timestamp() {
	date +"%T"
}

LOG_DIR=/home/alarm/logs
t_start=$(timestamp)

/home/alarm/QuadcopterCodebase/GroundSoftware/can_bringup.sh

msgs="sf11_bno055 flow ahrs_rp yaw_height v_z"
for m in $msgs; 
	do
		filename=$LOG_DIR/${m}_${t_start}.csv
		echo "Logging data on "$m" to "$filename;
		/home/alarm/QuadcopterCodebase/GroundSoftware/sensor_daq.py can0 $m $filename &
done 

