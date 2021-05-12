"""
Collection of F1TENTH-compatible autonomous control algorithms.
"""
import os
import json

# Fetch the root folder to specify absolute paths to the "include" files
ROOT = os.path.join(os.path.normpath(os.path.dirname(__file__)), "..")

with open(os.path.join(__file__, "", "metadata.json")) as f:
    metadata = json.load(f)

__title__ = metadata["__title__"]
__version__ = metadata["__version__"]
__description__ = metadata["__description__"]
__lead__ = metadata["__lead__"]
__email__ = metadata["__email__"]
__url__ = metadata["__url__"]