Robot Model
-----------

Let the state variables of an agent be:

Independent of environment, but dependent on time since last collision and FREE
or WALL:

- theta, theta-dot
    - arbitrary angle chosen as heading, and rotational velocity
- v (linear velocity)

velocities can be calculcated with a kalman filter (or other filter) from
trajectory. over what time window?

Transitions dependent on environment:
 
- time since last collision
    - could be typed, depending on sensor setup: wall or robot collisions
- FREE or WALL (or CORNER)
    - WALL if distance to nearest boundary below threshold

To-Dos
------

### Data Collection / Analysis

- do specific model identification / registration / find distinguishing features 
  by collecting data in specific scenarios or segmenting our current datasets into these 
  scenarios
- scenarios of interest:
    - in contact with wall
        - wall following duration? angle of departure?
    - moving in free space
    - corner interactions: reflex and non-reflex corners

### Hardware

- source and acquire electro-permanent magnets
- attach IMU to assembly and reliably detect collisions with environment; OR
- attach proximity detectors to assembly and use these to detect collisions


Deliverables
------------

### Data Analysis

- plot autocorrelation of speed for specific assemblies
    - at what time scale do the weaselballs start looking Brownian?
- plot distribution of speeds for specific assemblies
    - intuition: larger assemblies tend to "just sit"
- distribution of times in between collisions
    - analogue to mean free path
    - pressure?

### Predictive

- show evolution of location distribution from specific initial conditions
- given environment and agent type, return long-term distribution of:
    - times between collisions

### Estimation

- estimate how many other agents are in the space with you
    - collision frequency is inversely proportional to average assembly size (given
      fixed number of starting agents)

### Control

- feedback controller based on collision frequency
    - attach until collision frequency drops below threshold
    - control equilibrium # of agents

Misc
----

### Other possible modelling variables:

- distance from nearest boundary
- distance from nearest other robot
- type of assembly (how to estimate? communicate with neighbor?)
