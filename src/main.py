"""Module entrypoint. Importing the model registers it with the Viam SDK."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import asyncio

from viam.module.module import Module

import models.dabai_max  # noqa: F401

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
