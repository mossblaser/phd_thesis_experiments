clstr -s -N "$1" -J <(
  for n in `seq 1000`; do
    echo python $PWD/cost_function_experiment.py -r 50 \> $PWD/out_$n.csv;
  done
)

( head -n1 out_1.csv; tail -q -n+2 out_*.csv ) > out.csv
