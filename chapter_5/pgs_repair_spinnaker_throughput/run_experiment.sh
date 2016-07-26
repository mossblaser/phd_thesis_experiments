# Number of boards to use
NUM_BOARDS=(3)

# Numbers of dead links to add
DEAD_LINKS=(0 1 2 3 4 5 6 7 8 9 10)

# Use uniform and centroid traffic
CENTROIDS=("" "--centroids 3")

# Use uniform and HSS link faults
LINK_FAULTS=("" "--hss-links")

# Number of repeats for each configuration
REPEATS=100

# Number of three-board SpiNNaker machines to use at once.
MAX_AT_ONCE=20

clstr --status -N <(cat "$1" | head -n$MAX_AT_ONCE) -J <(
    for REPEAT in `seq $REPEATS`; do
        for DEAD in "${DEAD_LINKS[@]}"; do
            for BOARDS in "${NUM_BOARDS[@]}"; do
                for CENTROID in "${CENTROIDS[@]}"; do
                    for LINK_FAULT in "${LINK_FAULTS[@]}"; do
                        echo cd $PWD\; python saturate.py \
                                $BOARDS \
                                --trim \
                                -d $DEAD \
                                $CENTROID \
                                $LINK_FAULT \
                                ;
                    done
                done
            done
        done
    done
) >> results.csv
