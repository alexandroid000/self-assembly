# Weaselbot Simulation
A 2D Python simulation of autonomous n-sided polygonal robots with random motion. They may connect with one another to create larger systems of randomly moving robots.

### Getting Started
##### Dependencies
- python3
- pymunk

##### Executing Program
Run `python3 pymunk-simulation.py m n` where `m` is the number of units, `n` is the sides per unit.
Example: `python pymunk-simulation.py 64 6`
To quit, press "Esc".
To print to .png, press "p"
To change speed, press "1-9" (default=5)
To pause, press "Space"

### Description
This pygame simulation incorporates the pymunk library, which uses the Chipmunk
physics engine. It is meant to simulate robots that can connect with one another but otherwise have random motion. This is only meant to be a 2-Dimensional simulation.

### Bug Log
- Adding joints after initial start-up creates erratic behaviors
- - Configurations made in the initialization