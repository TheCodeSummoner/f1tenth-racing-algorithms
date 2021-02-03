"""
See the package's `__init__.py` docstring for more information.
"""
import os
import sys

# Make sure a local f1tenth package can be found and overrides any installed versions
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from f1tenth.waypoints_follower import run  # pylint: disable=import-outside-toplevel, wrong-import-position

if __name__ == "__main__":
    run()
