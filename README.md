PhD Thesis Experiment Scripts
=============================

This repository collects together the software and scripts and used to perform
the bulk of the experiments for my PhD thesis. Please note that certain
experimental programs were written in collaboration with others and that the
scripts themselves are of the `rough and ready' veriety. The underlying
software, however, should be of substantially greater quality and generally
lives elsewhere (links are included for each experiment).

Experiment scripts
------------------

* Chapter 4: Finding shortest path vectors in SpiNNaker's network
  - [Shortest path vector function runtime](./chapter_4/shortest_path_vector_runtime/)
* Chapter 5: Fault tolerant routing for SpiNNaker
  - [PGS-repair runtime](./chapter_5/pgs_repair_runtime/)
  - [PGS-repair routing table usage and congestion](./chapter_5/pgs_repair_tables_and_congestion/)
  - [PGS-repair SpiNNaker throughput](./chapter_5/pgs_repair_spinnaker_throughput/)
* Chapter 6: Placing applications in large SpiNNaker machines
  - [Cost function comparison](./chapter_6/cost_function_comparison/)
  - [Application benchmark placement performance](./chapter_6/benchmark_placement/)
  - [Nengo SpiNNaker experiments](./chapter_6/nengo_spinnaker_test/)
  - [Placement algorithm scalability](./chapter_6/scalability/)


Result data
-----------

Raw result data for my runs of these experiments can be found in the sources
for my thesis (and are used during the build process to generate result plots
and figures).

License
-------

Except where otherwise explicitly noted (and there are some exceptions!), the
code in this repository is released into the public domain. If for some reason
this is insufficient, you may also use this code under the terms of the MIT
license.  Attribution and citation of my PhD thesis would be appreciated but is
not required. A BibTeX citation you could use is:

    @phdthesis{heathcote16,
      author       = {Jonathan Heathcote}, 
      title        = {Building and operating large-scale SpiNNaker machines},
      school       = {The University of Manchester, School of Computer Science},
      year         = 2016
    }

