#!/usr/bin/env sh

set -e
set -u

kernel_name=$(uname -r)
time_now=$(date +%Y%m%d_%H%M%S)
out_dir="/tmp/rttest/${kernel_name}/${time_now}"
mkdir -p ${out_dir}

base_params="--interval=10000 --distance 0 --threads"
tune_params="--affinity --priority=98 --policy=fifo --mlockall"
timeseries_params="--duration 1m"
histogram_params="--duration 20m --quiet --histofall 20000"
stress_params="--vm 4 --hdd 4 --io 4 --cpu 4"

echo "Phase 0: saving params"
echo "Run params:" > "${out_dir}/params.txt"
echo "base_params: ${base_params}" >> "${out_dir}/params.txt"
echo "tune_params: ${tune_params}" >> "${out_dir}/params.txt"
echo "timeseries_params: ${timeseries_params}" >> "${out_dir}/params.txt"
echo "histogram_params: ${histogram_params}" >> "${out_dir}/params.txt"
echo "stress_params: ${stress_params}" >> "${out_dir}/params.txt"

echo "Phase 1: baseline = no params, no stress"
echo "Phase 1.1: time series"
cyclictest ${base_params} ${timeseries_params} > "${out_dir}/baseline.timeseries.txt"
echo "Phase 1.2: histogram"
cyclictest ${base_params} ${histogram_params} --histfile="${out_dir}/baseline.histogram.txt"

echo "Phase 2: tuned = params, no stress"
echo "Phase 2.1: time series"
cyclictest ${base_params} ${tune_params} ${timeseries_params} > "${out_dir}/tuned.timeseries.txt"
echo "Phase 2.2: histogram"
cyclictest ${base_params} ${tune_params} ${histogram_params} --histfile="${out_dir}/tuned.histogram.txt"

echo "Starting the stress"
stress ${stress_params} &
STRESS_PID=$!
echo "Stress started, parent PID: ${STRESS_PID}"

echo "Phase 3: regular = no params + stress"
echo "Phase 3.1: time series"
cyclictest ${base_params} ${timeseries_params} > "${out_dir}/regular.timeseries.txt"
echo "Phase 3.2: histogram"
cyclictest ${base_params} ${histogram_params} --histfile="${out_dir}/regular.histogram.txt"

echo "Phase 4: hardcore = params + stress"
echo "Phase 4.1: time series"
cyclictest ${base_params} ${tune_params} ${timeseries_params} > "${out_dir}/hardcore.timeseries.txt"
echo "Phase 4.2: histogram"
cyclictest ${base_params} ${tune_params} ${histogram_params} --histfile="${out_dir}/hardcore.histogram.txt"

echo "Killing the stress process and its children"
pstree $STRESS_PID -p -a -l | cut -d, -f2 | cut -d' ' -f1 | xargs kill -9

echo "Saving the results"
cp -r /tmp/rttest/* ~/rttests/results/
