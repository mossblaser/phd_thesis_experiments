SIZE=128
DISTANCE=8
FAN_OUTS=(1 2 4 8 16 32 64 128 256)
REPEATS=10
ALGOS=(sa hilbert rcm rand)

clstr --status -N "$1" -J <(
    for REPEAT in `seq $REPEATS`; do
        for FAN_OUT in ${FAN_OUTS[@]}; do
            for ALGO in ${ALGOS[@]}; do
                echo cd $PWD\; python scalability.py -W $SIZE --fan-out $FAN_OUT --distance $DISTANCE -p $ALGO
            done
        done
    done
) > results_fan.csv



