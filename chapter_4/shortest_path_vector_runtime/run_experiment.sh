#!/bin/bash

# Run a number of repetitions of the experiment with each algorithm.

gcc -O2 runtime.c algos.c -o runtime || exit 1

# Size of topology to use
WIDTH=240
HEIGHT=240

# Repeats per algorithm
REPS=50

# Algorithm names (in the order used in the code)
ALGOS=("INSEE Method" "IQ Method" "XYZ-Protocol")

echo "width,height,algo,runtime"

for REP in $(seq $REPS); do
	for ALGO in $(seq 0 $((${#ALGOS[@]} - 1))); do
		echo -n "$WIDTH,$HEIGHT,${ALGOS[$ALGO]},"
		( TIMEFORMAT="%E"; time ./runtime $ALGO $WIDTH $HEIGHT ) 2>&1 | tail -n1
	done
done
