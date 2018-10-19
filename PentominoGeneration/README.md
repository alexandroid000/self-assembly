# Polyomino Generator
A python program that prints all possible configurations for a polyomino with an arbitrary number of tiles.

### Getting Started
##### Dependencies
- python3
- numpy
##### Executing Program
Run `python3 PolyominoGenerator.py n` where `n` is the desired number of polyomino tiles.
For example, `python3 PolyominoGenerator.py 4` generates all tetrominos.

### Description
The PolyominoGenerator.py generates matrices of Xs for all unique configurations for a given number of polyomino tiles, along with a unique ID label based on movement around the perimeter of the configuration. A polyomino is a generalization of a pentomino that can have any number of tiles. This program only generates polyominos with square tiles and does not handle different shapes with the same outer perimeter.

This program uses a lexicographic ordering to ensure the same configuration is only printed once (though symmetries and transformations of configurations do appear). The program uses a recursive function that adds positive integers as "used" tiles, and uses negative integers to enumerate locations for future tiles to be placed. Any negative integers with magnitudes greater than the largest positive integer are changed to -1 to signify that this location cannot be used for future tiles. See in the textbook, these numbers are circled instead (this is the basis for the lexicographic ordering).