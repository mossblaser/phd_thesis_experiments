Shortest path vector function runtime
=====================================

The experiment from section 4.3.3 in my thesis to determine the relative
execution speed of various shortest path vector functions.

Usage:

    $ ./run_experiment.sh > results.csv

This experiment will take a long time to run. Reduce the `WIDTH` and `HEIGHT`
defined in `run_experiment.sh` to speed things up.

The `algos.c` file contains implementations of each shortest path function.

The code in this directory was largely written by me except for some functions
which are explicitly identified in the source code.

This code is known to work on Linux when built with GCC 4.8.3 and 6.1.1.
