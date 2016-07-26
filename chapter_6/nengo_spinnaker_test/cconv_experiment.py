import argparse
import logging
import nengo
from nengo import spa
import nengo_spinnaker
import numpy as np
from six import iteritems
import time



def label_net(network, prefixes=tuple()):
    for net in network.networks:
        label_net(net, prefixes + (net.label, ))

    prefix = ".".join(p or "?" for p in prefixes)
    for node in network.nodes:
        node.label = "{}.{}".format(prefix, node.label)


def make_model(n_dimensions, seed):
    """Create a model with the given parameters."""
    with spa.SPA(seed=seed) as model:
        # Create the inputs
        model.a = spa.State(dimensions=n_dimensions)
        model.b = spa.State(dimensions=n_dimensions)

        # Create the output
        model.c = spa.State(dimensions=n_dimensions)

        # Create the convolution
        model.cortical = spa.Cortical(spa.Actions("c = a * b"), synapse=0.01)

        # Add some input
        model.input = spa.Input(a='A', b='B')

    label_net(model)
    return model


def run_experiment(model, spinnaker, placer):
    """Run the experiment and return the results in a dictionary."""
    # Add probes for the output of the network
    with model:
        p_c = nengo.Probe(model.c.output, synapse=0.01)

    # Create the simulator
    if not spinnaker:
        sim = nengo.Simulator(model)
    else:
        # Configure all the Nodes as function of time Nodes
        nengo_spinnaker.add_spinnaker_params(model.config)

        for n in model.all_nodes:
            if n.output is not None:
                model.config[n].function_of_time = True

        # XXX
        import importlib
        model.config[nengo_spinnaker.Simulator].placer = \
            importlib.import_module("rig.place_and_route.place.{}".format(placer)).place

        sim = nengo_spinnaker.Simulator(model, use_spalloc=True, allocation_fudge_factor=2.0)

        # Get the current number of dropped packets
        dropped = sum(
            sim.controller.get_router_diagnostics(x, y).dropped_multicast
            for (x, y) in sim.controller.get_machine()
        )

    # Run the simulation
    try:
        sim.run(0.5)

        # Get the vocabulary
        vocab = model.get_output_vocab("c")
        AB = vocab.parse("A * B").v

        # Prepare the results for later analysis, recording the magnitude of the
        # error vector
        results = dict()
        results["output"] = np.sqrt(np.sum(np.square(AB - sim.data[p_c]), axis=1))
        results["times"] = sim.trange()

    finally:
        # Tidy up SpiNNaker and get the count of dropped packets
        if spinnaker:
            # Count the number of packets dropped during the simulation
            results["dropped_multicast"] = sum(
                sim.controller.get_router_diagnostics(x, y).dropped_multicast
                for (x, y) in sim.controller.get_machine()
            ) - dropped

            sim.close()

    return results


def run_all_experiments(n_dimensions, spinnaker=False, runs_per_scale=30,
                        runs_per_seed=1, filename=None, placer="sa"):
    """Run a large number of experiments."""
    # Initialise the results as empty lists
    data = {"n_dimensions": list(),
            "seed": list(),
            "times": None,
            "output": list()}
    if spinnaker:
        data["dropped_multicast"] = list()

    # Repeatedly build and run the experiments
    for n_dims in n_dimensions:
        for seed in range(runs_per_scale):
            for _ in range(runs_per_seed):
                model = make_model(n_dims, seed)
                try:
                    results = run_experiment(model, spinnaker, placer)

                    # Combine the results with the already stored results
                    data["n_dimensions"].append(n_dims)
                    data["seed"].append(seed)
                    for k, v in iteritems(results):
                        if k == "times":
                            if data["times"] is None:
                                data["times"] = v
                        else:
                            data[k].append(v)
                finally:
                    pass
                #except Exception as e:
                #    print("FAILED.")
                #    print(e)

    # Store all the data in Numpy arrays and write to file
    final_data = dict()
    for k in ("n_dimensions", "seed"):
        final_data[k] = np.array(data.pop(k), dtype=np.int)

    for k, v in iteritems(data):
        final_data[k] = np.array(v)

    if filename is None:
        filename = "cconv_{}_{}_{}_{}_{}_{}.npz".format(
            "nengo" if not spinnaker else "spinnaker",
            ",".join(map(str, n_dimensions)),
            placer,
            runs_per_scale,
            runs_per_seed,
            int(time.time())
        )

    np.savez_compressed(filename, **final_data)


if __name__ == "__main__":
    # Get the arguments from the command line
    parser = argparse.ArgumentParser()
    parser.add_argument("n_dimensions", nargs="+", type=int)
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--runs-per-seed", type=int, default=1)
    parser.add_argument("-s", "--spinnaker", action="store_true")
    parser.add_argument("--filename", type=str, default=None)
    parser.add_argument("-v", "--verbosity", action="count", default=0)
    parser.add_argument("-p", "--placer", type=str, default="sa")
    args = parser.parse_args()

    if args.verbosity == 1:
        logging.basicConfig(level=logging.INFO)
    if args.verbosity >= 2:
        logging.basicConfig(level=logging.DEBUG)

    # Run the experiment
    run_all_experiments(args.n_dimensions,
                        args.spinnaker,
                        args.runs,
                        args.runs_per_seed,
                        args.filename,
                        args.placer)
