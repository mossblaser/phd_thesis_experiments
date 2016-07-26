import random
import math
import time
import logging
import argparse
import sys

from collections import defaultdict

from rig.place_and_route import place, allocate, route, Machine, Cores, SDRAM
from rig.netlist import Net
from rig.routing_table import routing_tree_to_tables, minimise_tables
from rig.routing_table.remove_default_routes import minimise as remove_default_routes


from select_algo import placer

# Parse arguments
parser = argparse.ArgumentParser(description="Scalability experiment")
parser.add_argument("--print-header", action="store_true", default=False)
parser.add_argument("--width", "-W", type=int, default=None)
parser.add_argument("--height", "-H", type=int, default=None)
parser.add_argument("--fan-out", "-f", type=int, default=4)
parser.add_argument("--distance", "-d", type=float, default=3.0)
parser.add_argument("--verbose", "-v", action="count", default=0)
parser.add_argument("--placer", "-p", type=placer, default=(place, "default"))
parser.add_argument("--seed", "-s", type=int, default=random.getrandbits(32))
args = parser.parse_args()

if args.print_header:
	print("placer,width,height,fan_out,distance_sd,runtime,placed_net_length,manual_net_length,placed_total_entries,manual_total_entries,placed_max_entries,manual_max_entries")
	sys.exit(0)

if args.verbose >= 2:
	logging.basicConfig(level=logging.DEBUG)
elif args.verbose >= 1:
	logging.basicConfig(level=logging.INFO)

random.seed(args.seed)

width = args.width
if width is None:
	width = args.height
if width is None:
	parser.error("--width or --height not specified")
height = args.height if args.height else width

fan_out = args.fan_out
distance_sd = args.distance

place, placer_name = args.placer

# Construct network
vertices = {(x, y): object() for x in range(width) for y in range(height)}

nets = [Net(v, [vertices[(int(x+random.gauss(0, distance_sd))%width,
                          int(y+random.gauss(0, distance_sd))%height)]
                for _ in range(fan_out)])
        for (x, y),  v in vertices.items()]

vertices_resources = {v: {Cores: 1, SDRAM: 1024} for v in vertices.values()}

machine = Machine(int(math.ceil(width / 4.0)), int(math.ceil(height / 4.0)),
                  chip_resources={Cores:16, SDRAM:1024*1024})
#sys.stderr.write("Graph created...\n")

# Place-and-route
place_start = time.time()
placements = place(vertices_resources, nets, machine, [])
runtime = time.time() - place_start
#sys.stderr.write("Placed\n")

allocations = allocate(vertices_resources, nets, machine, [], placements)
#sys.stderr.write("Allocated\n")
routes = route(vertices_resources, nets, machine, [], placements, allocations)
#sys.stderr.write("Routed\n")

def count_table_entries(routes):
	# Work out how many table entries are necessary on each chip when
	# default-route removal is used.
	chip_entries = defaultdict(lambda: 0)
	for route in routes.values():
		for arriving, xy, leaving in route.traverse():
			if len(leaving) == 1 and tuple(leaving)[0] == arriving:
				# Default route!
				pass
			else:
				chip_entries[xy] += 1
	return (sum(chip_entries.values()), max(chip_entries.values()))
	
	# NB: Don't use actual routing table generator since the uncompressed tables
	# require essentially the same amount of memory as the routing tree and this
	# tips things over the edge for very large networks...
	#routing_tables = routing_tree_to_tables(
	#	routes,
	#	{n: (i, 0xFFFFFFFF) for i, n in enumerate(routes)})
	#sys.stderr.write("Tables generated\n")
	#routing_tables = minimise_tables(routing_tables, None, [remove_default_routes])
	#sys.stderr.write("Tables minimised\n")
	#
	## Return (total_entries, max_entries)
	#return (sum(len(t) for t in routing_tables.values()),
	#        max(len(t) for t in routing_tables.values()))

placed_net_length = sum(sum(1 for _ in route.traverse()) for route in routes.values())
#sys.stderr.write("Route length calculated\n")
placed_total_entries, placed_max_entries = count_table_entries(routes)

del routes

# 'Idealised' placement + routing
placements = {v: (x//4, y//4) for (x, y), v in vertices.items()}
allocations = allocate(vertices_resources, nets, machine, [], placements)
#sys.stderr.write("Idealised placement generated\n")
routes = route(vertices_resources, nets, machine, [], placements, allocations)
#sys.stderr.write("Idealised routing complete\n")

manual_net_length = sum(sum(1 for _ in route.traverse()) for route in routes.values())
manual_total_entries, manual_max_entries = count_table_entries(routes)

print(",".join(map(str, [
	placer_name,
	width, height,
	fan_out, distance_sd,
	runtime,
	placed_net_length, manual_net_length,
	placed_total_entries, manual_total_entries,
	placed_max_entries, manual_max_entries,
])))
