Application benchmark netlists
==============================

The benchmark netlists described in section 6.3.2 in my thesis. See the thesis
for details of where each benchmark comes from.

Each JSON file contains a dictionary which contains (at least) the following
keys:

* `vertices_resources`
* `nets`

The `vertices_resources` entry contains a dictionary `{vertex: {resource: value,
...}, ...}`. Each vertex has a unique ID and must consume some resources. The
"SDRAM" and "Cores" resource must be defined for each vertex.

The `nets` entry contains a list of lists `[source, [sink, ...], weight]`. The
`source` is the ID of the vertex sourcing a net, the sinks list enumerates
vertex IDs of sinks of the net and the weight field (which may be omitted and
defaults to 1.0) is a weight.

The description style is a simple JSON serialisation of the [Rig place-and-route
data structures](http://rig.readthedocs.io/en/stable/place_and_route.html).
