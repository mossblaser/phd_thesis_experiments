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


def make_model(n_dims, seed):
    """Create a model with the given parameters."""
    with spa.SPA(seed=seed) as model:
        model.vision = spa.State(dimensions=n_dims)
        model.phrase = spa.State(dimensions=n_dims)
        model.motor = spa.State(dimensions=n_dims)
        model.noun = spa.State(dimensions=n_dims, feedback=1.0,
                               feedback_synapse=0.01)
        model.verb = spa.State(dimensions=n_dims, feedback=1.0,
                               feedback_synapse=0.01)

        model.bg = spa.BasalGanglia(spa.Actions(
            'dot(vision, WRITE) --> verb=vision',
            'dot(vision, ONE+TWO+THREE) --> noun=vision',
            '0.5*(dot(vision, NONE-WRITE-ONE-TWO-THREE) + '
            'dot(phrase, WRITE*VERB)) --> motor=phrase*~NOUN',
        ))
        model.thal = spa.Thalamus(model.bg)

        model.cortical = spa.Cortical(spa.Actions(
            'phrase=noun*NOUN',
            'phrase=verb*VERB',
        ))

        # Create the input
        def input_vision(t):
            seq = "WRITE ONE NONE WRITE TWO NONE THREE WRITE NONE".split()
            return seq[int(t / 0.5) % len(seq)]

        model.input = spa.Input(vision=input_vision)

    label_net(model)
    return model


def run_experiment(model, spinnaker, placer):
    """Run the experiment and return the results in a dictionary."""
    # Add probes for the output for the network
    with model:
        p_motor = nengo.Probe(model.motor.output, synapse=0.03)
        p_noun = nengo.Probe(model.noun.output, synapse=0.03)
        p_verb = nengo.Probe(model.verb.output, synapse=0.03)
        p_actions = nengo.Probe(model.thal.actions.output, synapse=0.01)
        p_utility = nengo.Probe(model.bg.input, synapse=0.01)

    # Create the simulator
    if not spinnaker:
        sim = nengo.Simulator(model)
    else:
        # Configure all the Nodes as function of time Nodes
        nengo_spinnaker.add_spinnaker_params(model.config)

        for n in model.all_nodes:
            if n.output is not None:
                model.config[n].function_of_time = True
        
        import importlib
        model.config[nengo_spinnaker.Simulator].placer = \
            importlib.import_module("rig.place_and_route.place.{}".format(placer)).place

        sim = nengo_spinnaker.Simulator(model, use_spalloc=True)

        # Get the current number of dropped packets
        dropped = sum(
            sim.controller.get_router_diagnostics(x, y).dropped_multicast
            for (x, y) in sim.controller.get_machine()
        )

    # Run the simulation
    sim.run(4.5)

    # Get the results
    symbols = ("ONE", "TWO", "THREE", "NONE", "WRITE", "NOUN", "VERB")
    results = dict()
    results["times"] = sim.trange()

    def get_results(probe, obj):
        vocab = model.get_output_vocab(obj)
        v = np.zeros((len(symbols), vocab.dimensions))

        for i, sym in enumerate(symbols):
            v[i] = vocab[sym].v

        results[obj] = np.dot(v, sim.data[probe].T).T

    get_results(p_motor, "motor")
    get_results(p_noun, "noun")
    get_results(p_verb, "verb")
    results["actions"] = sim.data[p_actions]
    results["utilities"] = sim.data[p_utility]

    # Tidy up SpiNNaker and get the count of dropped packets
    if spinnaker:
        # Count the number of packets dropped during the simulation
        results["dropped_multicast"] = sum(
            sim.controller.get_router_diagnostics(x, y).dropped_multicast
            for (x, y) in sim.controller.get_machine()
        ) - dropped

        #print("> Dropped %d" % results["dropped_multicast"])

        sim.close()

    return results


def run_all_experiments(n_dimensions, spinnaker=False, runs_per_scale=30,
                        runs_per_seed=1, filename=None, placer="sa"):
    """Run a large number of experiments."""
    # Initialise the results as empty lists
    data = {"n_dimensions": list(),
            "seed": list(),
            "times": None,
            "motor": list(),
            "noun": list(),
            "verb": list(),
            "actions": list(),
            "utilities": list()}
    if spinnaker:
        data["dropped_multicast"] = list()

    # Repeatedly build and run the experiments
    for n_dims in n_dimensions:
        for seed in range(runs_per_scale):
            for _ in range(runs_per_seed):
                model = make_model(n_dims, seed)
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

    # Store all the data in Numpy arrays and write to file
    final_data = dict()
    for k in ("n_dimensions", "seed"):
        final_data[k] = np.array(data.pop(k), dtype=np.int)

    for k, v in iteritems(data):
        final_data[k] = np.array(v)

    if filename is None:
        filename = "parse_{}_{}_{}_{}_{}.npz".format(
            "nengo" if not spinnaker else "spinnaker",
            ",".join(map(str, n_dimensions)),
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
    parser.add_argument("--placer", type=str, default="sa")
    parser.add_argument("-v", "--verbosity", default=0, action="count")
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
