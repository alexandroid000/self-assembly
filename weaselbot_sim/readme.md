# Weaselbot Simulation
A 2D Python simulation of 

###Getting Started
#####Dependencies
- python3
- pymunk
#####Executing Program

###Description

This pygame simulation incorporates the pymunk library, which uses the Chipmunk
physics engine. However, adding joints after initial start-up creates extreme
behavior that I have had trouble debugging. Configurations made in the initialization
of the simulation do not have this extreme behavior.

Run `python3 pymunk-simulation.py m n` where `m` is the number of units, `n` is the sides per unit.
Example: `python pymunk-simulation.py 64 6`
To quit, press "Esc".
To print to .png, press "p"
To change speed, press "1-9" (default=5)
To pause, press "Space"

###Bug Log