# Self Assembly
A repository related to self-asssembly robot systems.

### Getting Started
##### Dependencies
- python 3.6
- numpy
- pymunk

### Modules
- **background_resources**: This module contains various examples and/or code snippets of the frameworks used throughout the project. It is used to keep basic code that can be used as a reference for the contributors of the project.
- **CAD**: This module is a folder that contains the CAD models of weazel-ball locomoted robots that are used for physical self-assembly runs.
- **dataAnalysis**: This module contains data analysis tools for analyzing data collected through physical self-assembly runs as well as simulated runs.
- **gazebo**: This module contains tools necessary for running a 3D simulation of the self-assembly robots.
- **open_cv**: This module contains tools for analyzing video data collected from physical runs of the self-assembly robots.
- **PentominoGeneration**: This module contains code that generates all possible configurations of a poly-amino (a square-tiled shaped) with some integer number of tiles.
- **PythonSimulation**: This module contains various code snippets from a historic 2D simulation of self-assembly robots.
- **weaselbot_sim**: This module simulates a 2D set of self-assembly robots.

### Motivation
This projects is meant to explore various aspects of self-assembly robot systems to gain an intuition on how these systems could be controlled. In this self-assembly exploration, robots are independent and have pseudo-random motion. Each robot has different connection rules that allow the system to autonomously and  create complex structures of robots. The aim is to determine what kinds of complex systems can be created from independent robots when their connection rules are controlled.

### Examples
![alt text](https://raw.githubusercontent.com/alexandroid000/self-assembly/assemblies.png)
![alt text](https://raw.githubusercontent.com/alexandroid000/self-assembly/connections.png)
![alt text](https://raw.githubusercontent.com/alexandroid000/self-assembly/open_cv_short.png)
![alt text](https://raw.githubusercontent.com/alexandroid000/self-assembly/open_cv_long.png)
