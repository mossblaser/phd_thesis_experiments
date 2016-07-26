SIZES=(9 12 16 24 32 48 64 96 128 192 256 384 512 768 1024)
REPEATS=20
ALGOS=(sa hilbert rcm rand)

clstr --status -N "$1" -J <(
    for REPEAT in `seq $REPEATS`; do
        for SIZE in ${SIZES[@]}; do
            for ALGO in ${ALGOS[@]}; do
                if [ "$SIZE" -le 512 -o "$ALGO" == "sa" ]; then
                    echo cd $PWD\; python scalability.py -W $SIZE -p $ALGO
                fi
            done
        done
    done
) >> results.csv
