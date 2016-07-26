import sys
import json
import time
import importlib
from collections import defaultdict

from rig.place_and_route.constraints import SameChipConstraint
from rig.place_and_route import Machine, Cores, SDRAM, allocate, route
from rig.routing_table import routing_tree_to_tables, minimise_tables
from rig.routing_table.remove_default_routes import minimise as remove_default_routes
from rig.routing_table import MinimisationFailedError
from rig.netlist import Net
from rig.links import Links

# Cairo PDF points to MM conversion
PT_PER_MM = 2.83464567


def read_json_netlist(filename):
    """Read a JSON netlist file.
    
    Returns
    -------
    vertices_resources : {vertex: {resource: quantitiy, ...}, ...}
    nets : [Net(source, sinks, weight), ...]
    constraints : [...]
    """
    with open(filename, "r") as f:
        netlist = json.load(f)
    
    vertices_resources = {
        v: {Cores: r["Cores"], SDRAM: r["SDRAM"]}
        for v, r in netlist["vertices_resources"].items()
    }
    
    nets = [Net(n[0], n[1], n[2] if len(n) > 2 else 1.0)
            for n in netlist["nets"]]
    
    constraints = [SameChipConstraint(vs) for vs in
                   netlist.get("same_chip_constraints", [])]
    
    return vertices_resources, nets, constraints


def create_machine(w=12, h=12, wrap_around=False):
    """Create a machine of a certain size with or without wrap-around links.
    """
    return Machine(w, h, chip_resources={Cores: 16, SDRAM:128*1024*1024},
                   # If non-wrapping add enough dead links to make the
                   # placers etc. presume torus connectivity isn't
                   # available.
                   dead_links={(x, y, link)
                               for x in range(w)
                               for y, link in [(0, Links.south),
                                               (h-1, Links.north)]}
                              if not wrap_around else set())

def place(algorithm, vertices_resources, nets, machine, constraints):
    """Place using a named algorithm.
    
    Returns
    -------
    placements, runtime
    """
    # Perform placement
    fn = importlib.import_module(
        "rig.place_and_route.place.{}".format(algorithm)).place
    before = time.time()
    placements = fn(vertices_resources, nets, machine, constraints)
    return placements, time.time() - before


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("netlist", help="JSON Netlist filename.", nargs="+")
    parser.add_argument("--placer", "-p", default=["sa"], nargs="+",
                        help="Placement algorithm(s).")
    parser.add_argument("--machine", "-m", nargs=2, type=int, default=(13, 13),
                        help="Machine dimensions")
    parser.add_argument("--torus", "-t", action="store_true", default=False,
                        help="Include torus links in machine?")
    parser.add_argument("--num-trials", "-n", type=int, default=5)
    args = parser.parse_args()
    
    machine = create_machine(args.machine[0], args.machine[1], args.torus)
    
    print("netlist,placer,runtime,total_cost,worst_case_chip_cost,total_entries,max_entries")
    for n in range(args.num_trials):
        for netlist in args.netlist:
            for placer in args.placer:
                #sys.stderr.write("Reading netlist...\n")
                vertices_resources, nets, constraints = \
                    read_json_netlist(netlist)
                #sys.stderr.write("Placing...\n")
                placements, place_time = place(placer,
                                               vertices_resources, nets,
                                               machine, constraints)
                #sys.stderr.write("Allocating...\n")
                allocations = allocate(vertices_resources, nets, machine, constraints, placements)
                #sys.stderr.write("Routing...\n")
                routes = route(vertices_resources, nets, machine, constraints, placements, allocations)
                
                # Calculate table usage. Try minimisation and if it fails just
                # use default route removal
                routing_tables = routing_tree_to_tables(
                    routes,
                    {n: (i, 0xFFFFFFFF) for i, n in enumerate(routes)})
                routing_tables = minimise_tables(routing_tables, None)
                #try:
                #    routing_tables = minimise_tables(routing_tables, 1023)
                #except MinimisationFailedError:
                #    routing_tables = minimise_tables(routing_tables, None, [remove_default_routes])
                total_entries = sum(len(t) for t in routing_tables.values())
                max_entries = max(len(t) for t in routing_tables.values())
                
                # Calculate cost due to paths
                cost = sum(net.weight * sum(1 for _ in tree.traverse())
                           for net, tree in routes.items())
                chip_cost = defaultdict(lambda: 0.0)
                for net, tree in routes.items():
                    for direction, xy, children in tree.traverse():
                        chip_cost[xy] += net.weight
                print(",".join(map(str, (
                    netlist,
                    placer,
                    place_time,
                    cost,
                    max(chip_cost.values()),
                    total_entries,
                    max_entries))))
