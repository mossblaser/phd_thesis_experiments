Placement algorithm scalability
===============================

The scalability experiments described in section 6.3.3 of my thesis. These
experiments attempt to place a simple 2D mesh into a SpiNNaker machine of
increasing size or with networks of increasing fan-out.

Usage:

    $ ./run_scalability.sh NODE_FILE
    $ ./run_fan_scalability.sh NODE_FILE

These scripts were written/hacked-together by me.

This experiment will take a long time to run on one machine and uses the
['clstr'](https://github.com/mossblaser/clstr) cluster management tool to run
the experiment across many machines.

This code is known to work on Linux using:

* Python v3.5.1
* `rig` v2.1.1
