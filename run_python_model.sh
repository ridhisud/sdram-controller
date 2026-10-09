#!/bin/bash
cd pymodel && python3 sdram_pymodel.py && python3 sdram_pymodel.py 100 && python3 make_figs.py
