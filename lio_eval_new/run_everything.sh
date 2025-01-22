#!/bin/bash

./run_kiss_icp.sh
sleep 3
./run_genz_icp.sh

echo "All experiments completed!"