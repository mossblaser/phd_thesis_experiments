Cost function comparison
========================

The experiment from section 6.2.2 in my thesis which measures the error in the
cost estimates produced by various cost functions against the NER router.

Usage:

    $ ./run_experiment.sh NODE_FILE

This code was written/hacked-together entirely by me.

This experiment will take a long time to run on one machine and uses the
['clstr'](https://github.com/mossblaser/clstr) cluster management tool to run
the experiment across many machines.

This code is known to work on Linux using:

* Python v3.5.1
* `rig` v2.1.1
