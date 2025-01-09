#!/bin/bash

./run_fast_lio.sh
sleep 3
./run_ig_lio.sh
sleep 3
./run_kiss_icp.sh
sleep 3
./run_genz_icp.sh
sleep 3
./run_dlio.sh
sleep 3
./run_dlo.sh
sleep 3
./run_liosam.sh
sleep 3

echo "All experiments completed!"