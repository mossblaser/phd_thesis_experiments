import random

from functools import partial
from collections import OrderedDict
from math import sqrt

from rig.place_and_route import Cores, route, Machine
from rig.netlist import Net
from rig.links import Links
from rig.geometry import shortest_mesh_path_length, shortest_torus_path_length

def random_placement(n_vertices, chips):
    """Generate a random placement of n_vertices connected by a net."""
    vertices = [object() for _ in range(n_vertices)]
    placements = {v: xy for v, xy in zip(vertices, random.sample(chips, n_vertices))}
    
    source = random.choice(vertices)
    sinks = list(set(vertices) - set([source]))
    net = Net(source, sinks)
    
    return net, placements


def routed_cost(net, placements, width, height, has_wrap_around_links):
    vertices_resources = {v: {} for v in placements}
    allocations = {v: {} for v in placements}
    machine = Machine(width, height)
    if not has_wrap_around_links:
        # Add enough links to mark the machine as dead
        for x in range(machine.width):
            machine.dead_links.add((x, 0, Links.south))
            machine.dead_links.add((x, machine.height - 1, Links.north))
    
    routes = route(vertices_resources, [net], machine, [], placements, allocations)
    
    return sum(1 for _ in routes[net].traverse()) - 1


def hpwl_cost(net, placements, width, height, has_wrap_around_links):
    if has_wrap_around_links:
        # When wrap-around links exist, we find the minimal bounding box and
        # return the HPWL weighted by the net weight. To do this the largest
        # gap between any pair of vertices is found::
        #
        #     |    x     x             x   |
        #                ^-------------^
        #                    max gap
        #
        # The minimal bounding box then goes the other way around::
        #
        #     |    x     x             x   |
        #      ----------^             ^---

        # First we collect the x and y coordinates of all vertices in the net
        # into a pair of (sorted) lists, xs and ys.
        x, y = placements[net.source]
        num_vertices = len(net.sinks) + 1
        xs = [x] * num_vertices
        ys = [y] * num_vertices
        i = 1
        for v in net.sinks:
            x, y = placements[v]
            xs[i] = x
            ys[i] = y
            i += 1

        xs.sort()
        ys.sort()

        # The minimal bounding box is then found as above.
        x_max_delta = 0
        last_x = xs[-1] - width
        for x in xs:
            delta = x - last_x
            last_x = x
            if delta > x_max_delta:
                x_max_delta = delta

        y_max_delta = 0
        last_y = ys[-1] - height
        for y in ys:
            delta = y - last_y
            last_y = y
            if delta > y_max_delta:
                y_max_delta = delta

        return ((width - x_max_delta) +
                (height - y_max_delta))
    else:
        # When no wrap-around links, find the bounding box around the vertices
        # in the net and return the HPWL weighted by the net weight.
        x1, y1 = x2, y2 = placements[net.source]
        for vertex in net.sinks:
            x, y = placements[vertex]
            x1 = x if x < x1 else x1
            y1 = y if y < y1 else y1
            x2 = x if x > x2 else x2
            y2 = y if y > y2 else y2

        return (x2 - x1) + (y2 - y1)

def sqrt_hpwl_cost(net, placements, width, height, has_wrap_around_links):
    return 0.5 * sqrt(len(net.sinks) + 1) * hpwl_cost(net, placements, width, height, has_wrap_around_links)

def hex_hpwl_cost(net, placements, width, height, has_wrap_around_links):
    costs = []
    for unused_dimension, x_dim, y_dim in ((0, 1, 2), (0, 2, 1), (2, 1, 0)):
        placements_t = {
            v: (((x, y, 0)[x_dim] - (x, y, 0)[unused_dimension]) % width,
                ((x, y, 0)[y_dim] - (x, y, 0)[unused_dimension]) % height)
            for v, (x, y) in placements.items()
        }
        costs.append(hpwl_cost(net, placements_t, width, height, has_wrap_around_links));
    return min(costs)

def sqrt_hex_hpwl_cost(net, placements, width, height, has_wrap_around_links):
    return 0.5 * sqrt(len(net.sinks) + 1) * hex_hpwl_cost(net, placements, width, height, has_wrap_around_links)


def star_cost(net, placements, width, height, has_wrap_around_links):
    if has_wrap_around_links:
        fn = partial(shortest_torus_path_length, width=width, height=height)
    else:
        fn = shortest_mesh_path_length
    
    if len(net.sinks) >= 1:
        return sum(fn((placements[net.source][0], placements[net.source][1], 0),
                      (placements[sink][0], placements[sink][1], 0))
                   for sink in net.sinks)
    else:
        return 0


if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument("--width", "-W", type=int, default=30)
    parser.add_argument("--height", "-H", type=int, default=30)
    parser.add_argument("--min-num-vertices", "-n", type=int, default=2)
    parser.add_argument("--max-num-vertices", "-N", type=int, default=30)
    parser.add_argument("--wrap", "-w", action="store_true", default=False)
    parser.add_argument("--repeats", "-r", type=int, default=100)
    args = parser.parse_args()
    
    width = args.width
    height = args.height
    chips = [(x, y) for x in range(width) for y in range(height)]
    wrap_around = args.wrap
    
    cost_functions = OrderedDict([
        ("routed", routed_cost),
        ("hpwl", hpwl_cost),
        ("sqrt_hpwl", sqrt_hpwl_cost),
        ("hex_hpwl", hex_hpwl_cost),
        ("sqrt_hex_hpwl", sqrt_hex_hpwl_cost),
        ("star", star_cost),
    ])
    
    print(",".join(["n_vertices"] + list(cost_functions.keys())))
    for _ in range(args.repeats):
        for n_vertices in range(args.min_num_vertices, args.max_num_vertices + 1):
            net, placements = random_placement(n_vertices, chips)
            print(",".join([str(n_vertices)] + [
                str(f(net, placements, width, height, wrap_around))
                for f in cost_functions.values()
            ]))
