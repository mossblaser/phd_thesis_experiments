PGS-repair runtime
==================

The experiment from section 5.3.4 in my thesis which measures the runtime cost
of PGS repair when routing under various fault rates, traffic patterns and
fault patterns.

Usage (local):

    $ ./run_experiment.sh

Usage (cluster):

    $ ./run_experiment.sh -N NODE_FILE

This experiment will take a long time to run and is probably not practical to
run on a single machine. The ['clstr'](https://github.com/mossblaser/clstr)
cluster management tool can be used to run the experiment across many machines.

The `average_mc.uniform.iterative.bkt.HR.c` file contains an implementation of
the PGS repair and NER algorithm and due to its colourful history is a bit of a
mess and includes functionality not relevant to the work in my thesis. This
program was largely developed by Javier Navaridas with the PGS repair component
added by me at a much later stage. This file is released under the terms of the
GPLv2.

This code is known to work on Linux when built with GCC 4.8.3 or 6.1.1.

