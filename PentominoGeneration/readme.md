6/7/2018
Authors: Austin Born (ajbon2@illinois.edu), John Born (jmborn2@illinois.edu)

The PolyominoGenerator.py generates matrices of Xs for all unique configurations for a given number of
polyomino units, along with a unique ID label based on movement around the perimeter of the configuration.
A polyomino is a generalization of a pentomino that can have any number of tiles.This is currently only 
for 4-sided units, and does not handle different shapes with the same outer perimeter.

To use the PolyominoGenerator.py, you must have Python 3 or greater installed. To run, use the Python
command to run PolyominoGenerator.py with an integer "n" for the number of units as an argument:
python .\PolyominoGenerator.py 1
python .\PolyominoGenerator.py 8
