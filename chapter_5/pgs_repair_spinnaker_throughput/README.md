PGS-repair SpiNNaker throughput
===============================

The experiment from section 5.3.6 in my thesis which measures the
maximum throughput before network saturation in SpiNNaker hardware when routed
by PGS repair.

Usage:

    $ ./run_experiment.sh NODE_FILE

With the exception of the `add_centroid_flows` function (see its comment), this
code was written/hacked-together entirely by me.

This experiment will take a long time to run and uses the
['clstr'](https://github.com/mossblaser/clstr) cluster management tool to run
the experiment across many machines.

This code is known to work on Linux using:

* `rig` v2.1.1
* `network_tester` v1.0.1
