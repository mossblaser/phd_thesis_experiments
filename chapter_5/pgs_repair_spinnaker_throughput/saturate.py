#!/usr/bin/env python

import random
import time

from itertools import chain, cycle

from spalloc import Job

from rig.geometry import to_xyz, minimise_xyz, shortest_torus_path_length
from rig.machine_control import MachineController
from rig.machine_control.consts import AppState
from rig.links import Links

from network_tester import Experiment, to_csv

def add_centroid_flows(e, si, cores, fan_out, num_centroids=3):
    """Add centroid connectivity to the experiment.
    
    This function was adapted from
    https://github.com/mundya/mundy-hpsr2016/tree/master/experiments written by
    Andrew Mundy and released under the BSD License.
    
    Parameters
    ----------
    e : Experiment
    si : SystemInfo
    cores : {(x, y, p): vertex, ...}
    num_centroids : int
    """
    # Compute the distance dependent probabilities - this is a geometric
    # distribution such that each core has a 50% chance of being connected to
    # each core on the same chip, 25% on chips one hop away, 12.5% on chips two
    # hops away, etc.
    p = 0.5
    probs = {d: p*(1 - p)**d for d in
             range(max(si.width, si.height))}

    p = 0.3
    dprobs = {d: p*(1 - p)**d for d in
              range(max(si.width, si.height))}

    chips = set((x, y) for (x, y, p) in cores)

    # Make the nets, each vertex is connected with distance dependent
    # probability to other cores.
    for source_coord, source in cores.items():
        # Convert source_coord to xyz form
        source_coord_xyz = minimise_xyz(to_xyz(source_coord[:-1]))

        # Add a number of centroids
        centroids = [minimise_xyz(to_xyz(xy))
                     for xy in random.sample(chips, num_centroids)]

        # Construct the sinks list
        sinks = list()
        for sink_coord, sink in cores.items():
            # Convert sink_coord to xyz form
            sink_coord = minimise_xyz(to_xyz(sink_coord[:-1]))

            # Get the path length to the original source
            dist = shortest_torus_path_length(source_coord_xyz, sink_coord,
                                              si.width, si.height)
            if random.random() < probs[dist]:
                sinks.append(sink)
                continue

            # See if the sink is connected to the centre of any of the
            # centroids.
            for coord in centroids:
                dist = shortest_torus_path_length(
                    coord, sink_coord, si.width, si.height
                )

                if random.random() < dprobs[dist]:
                    sinks.append(sink)
                    break

        # Add the flow
        sinks = random.sample(sinks, min(fan_out, len(sinks)))
        e.new_flow(source, sinks)

def add_uniform_flows(e, si, cores, fan_out):
    cores = list(cores.values())
    # Produce random connectivity
    for core in cores:
        e.new_flow(core, random.sample(cores, fan_out))


def add_uniform_link_faults(si, added_dead_links):
    links_to_kill = random.sample([(x,y,l) for (x,y,l) in si.links() if l < 3],
                                  added_dead_links)
    for x, y, link in links_to_kill:
        si[(x, y)].working_links.remove(link)
        dx, dy = link.to_vector()
        x2 = (x + dx) % si.width
        y2 = (y + dy) % si.width
        if (x2, y2) in si:
            si[(x2, y2)].working_links.discard(link.opposite)


def add_hss_link_faults(si, added_dead_links):
    # Group links by HSS link
    link_vectors = []
    for x, y in [(0, 0), (4, 8), (8, 4)]:
        link_vectors += [
            (x, y, ((Links.south, 1, 0), (Links.south_west, 0, 0))),
            (x, y, ((Links.south_west, 0, 0), (Links.west, 0, 1)))
        ]
    for x, y in [(0, 4), (4, 0), (8, 8)]:
        link_vectors += [
            (4, 0, ((Links.south, 0, 0), (Links.east, 1, 1))),
        ]
    
    # Enumerate all the working HSS links
    hss_links = []
    for bx in range(0, si.width, 12):
        for by in range(0, si.height, 12):
            for sx, sy, directions in link_vectors:
                links = []
                x = bx + sx
                y = by + sy
                for _, (link, dx, dy) in zip(range(8), cycle(directions)):
                    if (x, y, link) in si:
                        links.append((x, y, link))
                    x += dx
                    y += dy
                if links:
                    hss_links.append(links)
    
    # Kill randomly selected HSS links
    random.shuffle(hss_links)
    cur_hss_link = []
    for _ in range(added_dead_links):
        if not cur_hss_link:
            cur_hss_link = hss_links.pop()
        x, y, link = cur_hss_link.pop()
        
        si[(x, y)].working_links.remove(link)
        dx, dy = link.to_vector()
        x2 = (x + dx) % si.width
        y2 = (y + dy) % si.width
        if (x2, y2) in si:
            si[(x2, y2)].working_links.discard(link.opposite)


def run_experiment(hostname, trim_edges=False, fan_out=16, timestep_us=10.0,
                   duration=0.1,
                   min_packets_per_second_per_chip=1000.0,
                   max_packets_per_second_per_chip=1000000.0,
                   num_steps=100,
                   saturation_threshold=0.95,
                   added_dead_links=0,
                   num_centroids=None,
                   hss_links=False):
    e = Experiment(hostname)
    si = e.system_info
    
    # Straighten-off the edges of a spalloced machine.
    if trim_edges and si.width > 8 and si.height > 8:
        x_range = (4, si.width - 4)
        y_range = (4, si.height - 4)
        for x in chain(range(4), range(si.width-4, si.width)):
            for y in range(si.height):
                si.pop((x, y), None)
        for x in range(si.width):
            for y in chain(range(4), range(si.height-4, si.height)):
                si.pop((x, y), None)
    else:
        x_range = (0, si.width)
        y_range = (0, si.height)
    
    # Kill some links
    if hss_links:
        add_hss_link_faults(si, added_dead_links)
    else:
        add_uniform_link_faults(si, added_dead_links)
    

    # Put a Core on every idle core
    cores = {(x, y, p): e.new_core(x, y)
             for x, y, p, state in si.cores()
             if state == AppState.idle}
    
    # Add flows
    if num_centroids is not None:
        add_centroid_flows(e, si, cores, fan_out, num_centroids)
    else:
        add_uniform_flows(e, si, cores, fan_out)
    
    num_chips = float(len(si))
    num_cores = float(len(cores))
    
    # Setup experimental parameters
    e.timestep = timestep_us * 1e-6
    e.warmup = 0.05
    e.cooldown = 0.05
    e.duration = duration
    
    # Scale the traffic over the course of the experiment
    for step in range(num_steps):
        with e.new_group() as g:
            perc = (step + 1) / float(num_steps)
            min_ppspc = min_packets_per_second_per_chip
            max_ppspc = max_packets_per_second_per_chip
            ppspc = ((max_ppspc - min_ppspc) * perc) + min_ppspc
            
            actual_max_pps = num_cores / e.timestep
            actual_max_ppspc = actual_max_pps / num_chips
            
            assert actual_max_ppspc >= ppspc, \
                "Can't generate {} packets per second per chip, " \
                "can only manage {}".format(ppspc, actual_max_ppspc)
            
            e.probability = ppspc / actual_max_ppspc
            g.add_label("packets_per_second_per_chip", ppspc)
    
    # Set up recording
    e.record_sent = True
    e.record_received = True
    
    # Run the experiment
    results = e.run(ignore_deadline_errors=True)
    
    # Find saturation point
    totals = results.totals()
    saturation_point = 0.0
    for ppspc, accepted in zip(totals["packets_per_second_per_chip"],
                               totals["received"] / totals["ideal_received"]):
        if accepted > saturation_threshold:
            saturation_point = ppspc
        else:
            break
    else:
        # Did not saturate!
        saturation_point = None
    
    return num_chips, num_cores, saturation_point

if __name__=="__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument("num_boards", type=int)
    parser.add_argument("--print-header",
                        action="store_true", default=False)
    parser.add_argument("--trim-edges", "-t",
                        action="store_true", default=False)
    parser.add_argument("--fan-out", "-f", type=int, default=16)
    parser.add_argument("--timestep", "-T", type=float, default=10.0,
                        help="microseconds")
    parser.add_argument("--duration", "-D", type=float, default=0.1)
    parser.add_argument("--min-packets-per-second-per-chip", "-p", type=float,
                        default=1000.0)
    parser.add_argument("--max-packets-per-second-per-chip", "-P", type=float,
                        default=500000.0)
    parser.add_argument("--num-steps", "-s", type=int, default=100)
    parser.add_argument("--saturation-threshold", "-S", type=float, default=0.95)
    parser.add_argument("--verbose", "-v", action="count", default=0)
    parser.add_argument("--added-dead-links", "-d", type=int, default=0)
    parser.add_argument("--centroids", "-c", type=int, default=None)
    parser.add_argument("--hss-links", "-H", action="store_true", default=False)
    args = parser.parse_args()
    
    if args.print_header:
        import sys
        print("added_dead_links,centroids,hss_links,num_chips,num_cores,saturation_point")
        sys.exit(0)
    
    import logging
    if args.verbose >= 2:
        logging.basicConfig(level=logging.DEBUG)
    elif args.verbose >= 1:
        logging.basicConfig(level=logging.INFO)
    
    with Job(args.num_boards) as j:
        time.sleep(5)
        
        # Boot the machine
        mc = MachineController(j.hostname)
        mc.boot()
        #mc.discover_connections()
        
        # Run the experiment
        num_chips, num_cores, saturation_point = \
            run_experiment(mc,
                           args.trim_edges,
                           args.fan_out,
                           args.timestep,
                           args.duration,
                           args.min_packets_per_second_per_chip,
                           args.max_packets_per_second_per_chip,
                           args.num_steps,
                           args.saturation_threshold,
                           args.added_dead_links,
                           args.centroids,
                           args.hss_links)
        print(",".join(map(str, (
            args.added_dead_links,
            args.centroids,
            args.hss_links,
            num_chips,
            num_cores,
            saturation_point
        ))))
