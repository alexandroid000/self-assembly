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

### Data Collection / Processing

We have some specific states of interest, for discrete-state model building,
and some observable quantities associated with each. By looking at the
distributions of these quantities for each assembly type, we hope to identify
some distinguishable and useful dynamical properties of each assembly type
(quantifying our intuition that they move differently).

- identify when robot is in contact with wall
    - from this data, would be nice to get distributions per assembly type for:
        - duration of contact with wall
        - angle of departure when robot moves back into free space
- moving in free space
    - distribution of free path lengths (time, space) and shapes
- corner interactions: reflex and non-reflex corners
    - distribution of time spent in corner
    - distribution of angles of departure

**Implementation:** We can segment existing data (manually? Could write a
script to do this while watching fast forwarded video?) or collect new data 
in these states (probably easier)

### Trajectory Analysis

- extract environment polygon from video (save at top of text file, in same
  frame as trajectory points)
- Right now we can extract (t,x,y) trajectory points. We would like to also have:
    - orientation
    - assembly type
    - velocity (linear and angular, from Kalman filter)
    - distance from nearest wall
    - distance from nearest other assembly
    - time since last collision with wall
    - time since last collision with other assembly

**Implementation:** The first two (orientation and assembly type) probably need
to be extracted from video frames, or from the state returned after circles
are identified in the video frame. We can downsample in time a bit probably if
this becomes prohibitively expensive computationally. The rest we can extract
from the trajectory point text files (expect we might need to also label collision 
events from the video to get high accuracy on those).

### Hardware

- source and acquire electro-permanent magnets (ask steve or MIT people?)
 
- attach IMU to assembly and reliably detect collisions with environment; OR
- attach proximity detectors to assembly and use these to detect collisions
 
Deliverables
------------

### Data Analysis

- plot autocorrelation of speed for specific assemblies
    - at what time scale do the weaselballs start looking Brownian?
    - What are the different time scales for different assembly types?
- plot distribution of speeds for specific assemblies
    - intuition: larger assemblies tend to "just sit"
- distribution of times in between collisions
    - analogue to mean free path
    - pressure? Alli is working on a thermodynamic interpretation

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
    - control equilibrium # (and type?) of agents

Misc
----

### Other possible modelling / control variables:

- distance from nearest boundary; distance from nearest other robot
    - suspect these are not as useful as time
- type of assembly (how to estimate? communicate with neighbor?)
