NETLISTS=(*.json)
REPEATS=1000
ALGOS=(sa hilbert rcm rand)

clstr --status -N $1 -J <(
    for REPEAT in `seq $REPEATS`; do
        echo cd $PWD\; python score.py ${NETLISTS[@]} -n 1 -p ${ALGOS[@]}
    done
) > results_score.csv




