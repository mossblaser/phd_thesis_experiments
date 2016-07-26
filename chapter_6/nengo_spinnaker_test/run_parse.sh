cd "$(dirname "$0")"

N_REPEATS=100
N_DIMS=512
PLACERS=(sa hilbert rcm rand)

clstr --status -N <(head -n40 "$1") -J <(
	for NUM in `seq $N_REPEATS`; do
		for PLACER in "${PLACERS[@]}"; do
			echo cd $PWD\; python parse_experiment.py \
				$N_DIMS \
				--spinnaker \
				--filename parse_${N_DIMS}_${PLACER}_${NUM}.npz \
				--runs 1 \
				--runs-per-seed 1\
				--placer $PLACER\
				;
		done
	done
)


(
	echo "placer,dropped_multicast"
	for NUM in `seq $N_REPEATS`; do
		for PLACER in "${PLACERS[@]}"; do
			python get_result.py "parse_${N_DIMS}_${PLACER}_${NUM}.npz" "${PLACER}"
		done
	done
) > results_parse.csv

