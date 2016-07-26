"""Print the dropped multicast count from a Nengo SpiNNaker experiment result
file.
"""
import sys
import numpy

print(",".join(sys.argv[2:]+[str(numpy.load(sys.argv[1])["dropped_multicast"][0])]))
