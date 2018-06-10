'''
This file is Austin Born's (ajborn2@illinois.edu) second
attempt at a pygame simulation (this time with pymunk for the
physics). Adjustments made here are not necessarily reflective 
of the current state of the simulation, but different changes are tested here.

To run, type: "python sandbox2-ajborn2.py n m" where n = number of units, m = sides per unit
To quit, press "Esc"

TODO:
- Deal with physics, erm, difficulties
    - Currently using PivotJoint for connections, but
      with the randomly-added forces, this can lead to extreme rotational velocities
'''

__version__ = "$Id:$"
__docformat__ = "reStructuredText"

# Python library import
import pygame
import random
import math
import sys
from pygame.locals import *
from pygame.color import *
import pymunk
import pymunk.pygame_util
from pymunk import Vec2d

# Declare global colors
# COLOR = (R, G, B) => 0-255 for each
GRAY = (200, 200, 200)   

# Collision type for weaselballs
COLLTYPE_UNIT = 1

# Collision callback function to constrain shapes
def addJoints(arbiter, space, data):
    # If shapes are within constrained range, add
    # a PivotJoint to the point(s) of contact
    marginMultiplier = 0.9
    bot1, bot2 = arbiter.shapes
    if ((bot1.body.position.x-bot2.body.position.x)**2 + (bot1.body.position.y-bot2.body.position.y)**2) < (2*r*marginMultiplier)**2:
        #space.add(pymunk.PinJoint(bot1.body, bot2.body))
        for c in arbiter.contact_point_set.points:
            space.add(pymunk.PivotJoint(bot1.body, bot2.body, c.point_a))

# Main function body
if __name__ == '__main__':
    # Initialize game
    pygame.init()
    running = True
    weaselballNum = int(sys.argv[1])
    sides = int(sys.argv[2])

    # Set display screen
    (screenw, screenh) = (400, 500)
    screen = pygame.display.set_mode((screenw, screenh))
    screen.fill(GRAY)
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    # Create game clock
    clock = pygame.time.Clock()

    # Instantiate pymunk space
    space = pymunk.Space()

    # Instantiate walls
    walls = [pymunk.Segment(space.static_body, (0, 0), (0, screenh), 1),
        pymunk.Segment(space.static_body, (0, screenh), (screenw, screenh), 1.0),
        pymunk.Segment(space.static_body, (screenw, screenh), (screenw, 0), 1.0),
        pymunk.Segment(space.static_body, (screenw, 0), (0, 0), 1.0)]  
    for line in walls:
        line.elasticity = 1
    space.add(walls)

    # Calculate dimensions for each instantation box
    splitNum = math.ceil(math.sqrt(weaselballNum))
    splitLenw = int(screenw/splitNum)
    splitLenh = int(screenh/splitNum)
    
    # Instantiate weaselball vertices and constants
    mass = 1000
    r = 20
    angle = (2*math.pi)/sides
    vertices = []
    for i in range(sides):
        vertices.append((r*math.cos((i+0.5)*angle),r*math.sin((i+0.5)*angle)))

    # Instantiate weaselball surface shapes and physics bodies
    # with random color and x- and y-positions
    for i in range(weaselballNum):
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255), 1)
        randx = random.randint(int(i%splitNum)*splitLenw+r, (int(i%splitNum)+1)*splitLenw-r)
        randy = random.randint(math.floor(i/splitNum)*splitLenh+r, (math.floor(i/splitNum)+1)*splitLenh-r)
        inertia = pymunk.moment_for_poly(mass, vertices)
        body = pymunk.Body(mass, inertia)
        body.position = randx, randy
        body.velocity = (0,0)
        shape = pymunk.Poly(body, vertices)
        shape.collision_type = COLLTYPE_UNIT
        shape.elasticity = 0.6
        shape.friction = 0.9
        shape.color = color
        space.add(body, shape)

    # Instantiate collision handler
    ch = space.add_collision_handler(COLLTYPE_UNIT, COLLTYPE_UNIT)
    ch.data["surface"] = screen
    ch.post_solve = addJoints

    # Counter for random motion
    counter = 0

    while running:
        # Handle external events
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                running = False
            elif event.type == KEYDOWN and event.key == K_p:
                pygame.image.save(screen, "bouncing_balls.png")
        
        # Clear screen
        screen.fill(GRAY)        

        # Randomized motion by applying forces at regular intervals
        if counter == 50:
            for bot in space.shapes:
                if isinstance(bot, pymunk.Poly) and bot.body != None:
                    bot.body.apply_force_at_local_point((random.uniform(-0.01, 0.01), random.uniform(-0.01, 0.01)), (0,0))
            counter = 0
        else:
            counter += 1

        # Update physics
        space.step(60)

        # Necessary for drawing shapes
        space.debug_draw(draw_options)
        
        # Update screen image
        pygame.display.flip()

        # Increment clock
        clock.tick(60)

        # Display frames/second as the display caption
        pygame.display.set_caption("fps: " + str(clock.get_fps()))