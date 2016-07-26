Nengo SpiNNaker experiments
===========================

The Nengo SpiNNaker experiments described in section 6.3.2 of my thesis. These
experiments run standard Nengo SpiNNaker benchmark programs repeatedly,
modified to use alternative placement algorithms.

Usage:

    $ ./run_cconv.sh NODE_FILE
    $ ./run_parse.sh NODE_FILE

The experiment scripts were originally written by Andrew Mundy who in turn
adapted benchmarks written by the Nengo developers. These are licensed under
the GPLv2.

This experiment will take a long time to run on one machine and uses the
['clstr'](https://github.com/mossblaser/clstr) cluster management tool to run
the experiment across many machines.

This code is known to work on Linux using:

* Python v2.7.11 (NB: Known not to work on Python 3)
* `rig` v2.1.1
* Nengo a2b6adf (branch `master` at the time)
* Nengo SpiNNaker bd15ac8 (branch `more-filter-parallelism-with-FIQ-hack` at the time)
* `spalloc` v0.2.5
