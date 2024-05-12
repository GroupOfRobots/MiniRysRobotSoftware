#!/usr/bin/env sh

set -e
# Don't `set -u` - causes problems with `ros2 run`
# set -u

if [ $# -ne 1 ]; then
	echo "Usage: $0 <arg>"
	echo "arg:"
	echo "\tvr"
	echo "\tmotors"
	echo "\tfan"
	echo "\tcs"
	echo "\tcs_rttest"
	echo "\tcs_rttest_rt"
	echo "\tcs_rttest_stress"
	echo "\tcs_rttest_rt_stress"
	exit 1
fi

. ~/minirys_ws/install/setup.sh

time_now=$(date +%Y%m%d_%H%M%S)
out_dir=~/rttests/results/ros/$(uname -r)
stress_params="--vm 4 --hdd 4 --io 4 --cpu 4 --backoff 10000000 --timeout 1200"

case $1 in
	'vr')
		ros2 launch minirys_ros2 minirys_vr.launch.py
	;;
	'motors')
		ros2 run minirys_ros2 motors --ros-args --remap __ns:=/minirys
	;;
	'fan')
		ros2 run minirys_ros2 fan --ros-args --remap __ns:=/minirys
	;;
	'cs')
		ros2 launch minirys_ros2 minirys_cs.launch.py
	;;
	'cs_rttest')
		mkdir -p ${out_dir}/cs_rttest
		ros2 launch minirys_ros2 minirys_cs_rttest.launch.py
		cp /tmp/minirys_* ${out_dir}/cs_rttest/
	;;
	'cs_rttest_rt')
		mkdir -p ${out_dir}/cs_rttest_rt
		ros2 launch minirys_ros2 minirys_cs_rttest_rt.launch.py
		cp /tmp/minirys_* ${out_dir}/cs_rttest_rt/
	;;
	'cs_rttest_stress')
		stress ${stress_params} &
		STRESS_PID=$!
		mkdir -p ${out_dir}/cs_rttest_stress
		ros2 launch minirys_ros2 minirys_cs_rttest.launch.py
		cp /tmp/minirys_* ${out_dir}/cs_rttest_stress/
		pstree $STRESS_PID -p -a -l | cut -d, -f2 | cut -d' ' -f1 | xargs kill -9
	;;
	'cs_rttest_rt_stress')
		stress ${stress_params} &
		STRESS_PID=$!
		mkdir -p ${out_dir}/cs_rttest_rt_stress
		taskset -c 1 ros2 launch minirys_ros2 minirys_cs_rttest_rt.launch.py
		cp /tmp/minirys_* ${out_dir}/cs_rttest_rt_stress/
		pstree $STRESS_PID -p -a -l | cut -d, -f2 | cut -d' ' -f1 | xargs kill -9
	;;
	*)
		echo "unrecognized command"
		exit 2
	;;
esac
