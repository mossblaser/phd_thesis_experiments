# Usage:
#
#    $ # Run locally
#    $ ./run.sh
#
#    $ # Run on a cluster of machines (using 'clstr')
#    $ ./run.sh -N NODE_LIST

if [ "$1" == "-N" ]; then
	cluster_file="$2"
	shift 2
else
	cluster_file=""
fi

REPEATS=1000

# Compile the experiment for each combination of fault mode and traffic pattern
for spinn_link_fail in "" "SPINN_LINK_FAIL"; do
	for centroids in "" 3; do
		postfix=""
		flags="-DFIRST_ALG=3 -DLAST_ALG=3"
		
		[ -n "$spinn_link_fail" ] && postfix="$postfix,spinn_link" || postfix="$postfix,uniform"
		[ -n "$spinn_link_fail" ] && flags="$flags -D SPINN_LINK_FAIL"
		
		[ -n "$centroids" ] && postfix="$postfix,cent$centroids" || postfix="$postfix,uniform"
		[ -n "$centroids" ] && flags="$flags -D CENTROIDS=$centroids"
		
		gcc -O2 -o experiment.baseline$postfix,DIJKSTRA average_mc.uniform.iterative.bkt.HR.c pqueue.c -lm $flags -D DIJKSTRA "$@"
	done
done

# Determine the most appropriate cluster management application
if [ -n "$cluster_file" ]; then
	# If a list of hosts was given, run the experiments on that cluster
	parallel_script="python clstr.py -s -vv -A active_nodes -N $cluster_file -J"
elif which parallel 2>/dev/null >/dev/null; then
	# If GNU parallel is installed, use that to run the script on the local
	# machine
	parallel_script="parallel -a"
else
	# Failing all of the above, just run everything serially
	parallel_script="bash"
fi

# Run all experiments in parallel
$parallel_script <(
	for repeat_num in `seq 0 $(($REPEATS - 1))`; do
		for f in experiment.*; do
			[ -x "$f" ] && echo 'cd '"$PWD"'; ./'"$f"' '$repeat_num' | gzip > "output.'"$f"','$repeat_num'"'
		done
	done
)

# Convert output into a valid CSV and prefix each file's results with the experiment name
(
	# Print the header
	echo -n "experiment,fault_model,dist,postprocess,seed,"
	gunzip $(ls output.experiment.* | head -n1) -c | head -n1
	
	# Print the body of the data
	for f in output.experiment.*; do
		gunzip "$f" -c \
			| grep -vE '^dest|^[ ]*$' \
			| sed -re "s/^(.*)$/${f:18},\\1/"
	done
) | gzip > output.csv.gz

