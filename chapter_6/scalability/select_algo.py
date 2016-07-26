import importlib
import argparse

def placer(name):
    try:
        mod = importlib.import_module(
            "rig.place_and_route.place.{}".format(name))
    except ImportError:
        raise argparse.ArgumentTypeError(
            "{:r} is not a valid placement algorithm".format(name))

    return (mod.place, name)
