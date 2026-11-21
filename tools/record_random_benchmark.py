#!/usr/bin/env python3
"""Record ten default official eight-drone random-obstacle trials and Eq. (45)."""
import sys

from record_wall_benchmark import main


if __name__ == '__main__':
    sys.exit(main(default_scene='random', default_runs=10))
