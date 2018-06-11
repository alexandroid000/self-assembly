'''
This file is Austin Born's (ajborn2@illinois.edu) second
attempt at a pygame simulation (this time with pymunk for the
physics). Adjustments made here are not necessarily reflective 
of the current state of the simulation, but different changes are tested here.

To run, type: "python sandbox2-ajborn2.py n m" where n = number of units, m = sides per unit
    Note: I recommend "python sandbox2-ajborn2.py 64 6"
To quit, press "Esc"
To print to .png, press "p"

TODO:
- Deal with physics, erm, difficulties
    - Currently using PivotJoint for connections, but with the randomly-added forces, 
        this can lead to extreme angular velocities
    - Maybe find a way to apply torque to counteract angular velocity
- Play with type of joint
- Play with elasticity settings
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

# Global constants
COLLTYPE_UNIT = 1                   # Collision type for weaselballs
proximityMargin = 2                 # extra margin for imperfectly-aligned shapes to connect
(screenw, screenh) = (400, 500)     # screen dimensions
dampingConstant = 0.99996           # Amount of velocity preserved per second (0.99996 = 0.004% loss per second)
mass = 100                          # weaselball mass
r = 15                              # weaselball radius from center to each vertex
elasticityConstant = 0.1            # weaselball elasticity
frictionConstant = 0.9              # weaselball friction
dt = 60                             # clock speed multiplier (default=60)
forceMagnitude = 0.0005             # magnitude of min and max range of random linear force applied to weaselballs
forcefreq = 50                      # frequency at which linear force is applied

# Collision callback function to constrain shapes
def addJoints(arbiter, space, data):
    # If shapes are within proximity and colliding at 2
    # points, add a PivotJoint to each point of contact
    proximityValue = (2*math.cos(angle/2)*r)**2 + (math.sin(angle/2)*r)**2 + proximityMargin
    bot1, bot2 = arbiter.shapes
    if (bot1, bot2) not in joints:
        joints[(bot1, bot2)] = False
    if joints[(bot1, bot2)] == False:
        if ((bot1.body.position.x-bot2.body.position.x)**2 + (bot1.body.position.y-bot2.body.position.y)**2) < proximityValue:
            #space.add(pymunk.PinJoint(bot1.body, bot2.body))
            if len(arbiter.contact_point_set.points) == 2:
                for c in arbiter.contact_point_set.points:
                    space.add(pymunk.PivotJoint(bot1.body, bot2.body, c.point_a))
                    # rest_length = math.sqrt((c.point_a[0]-c.point_b[0])**2 + (c.point_a[1]-c.point_b[1])**2)
                    # stiffness = 1000
                    # damping = 100
                    # space.add(pymunk.DampedSpring(bot1.body, bot2.body, c.point_a+(10,10), c.point_b, rest_length, stiffness, damping))
                    # Print statement for debugging
                    print("joint added at:"+str(c.point_a)+"for "+str(bot1)+"and "+str(bot2))
                joints[(bot1, bot2)] = True

# Main function body
if __name__ == '__main__':

    # Initialize game
    pygame.init()
    running = True
    weaselballNum = int(sys.argv[1])
    sides = int(sys.argv[2])

    # Create display screen
    screen = pygame.display.set_mode((screenw, screenh))
    screen.fill(GRAY)
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    # Create game clock
    clock = pygame.time.Clock()

    # Create pymunk space
    space = pymunk.Space()
    space.damping = dampingConstant

    # Instantiate walls
    walls = [pymunk.Segment(space.static_body, (0, 0), (0, screenh), 1),
        pymunk.Segment(space.static_body, (0, screenh), (screenw, screenh), 1),
        pymunk.Segment(space.static_body, (screenw, screenh), (screenw, 0), 1),
        pymunk.Segment(space.static_body, (screenw, 0), (0, 0), 1)]  
    for line in walls:
        line.elasticity = 1
    space.add(walls)

    # Calculate dimensions for each spawn box
    splitNum = math.ceil(math.sqrt(weaselballNum))
    splitLenw = int(screenw/splitNum)
    splitLenh = int(screenh/splitNum)
    
    # Instantiate weaselball vertices and constants
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
        shape.elasticity = elasticityConstant
        shape.friction = frictionConstant
        shape.color = color
        space.add(body, shape)

    # Instantiate collision handler
    ch = space.add_collision_handler(COLLTYPE_UNIT, COLLTYPE_UNIT)
    ch.data["surface"] = screen
    ch.post_solve = addJoints

    # Instantiate dictionary of joints
    joints = {}

    # Counter for random motion
    counter = 0

    # Game loop
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

        # Randomized motion by applying a random force on each unit's center
        if counter == forcefreq:
            for bot in space.shapes:
                if isinstance(bot, pymunk.Poly) and bot.body != None:
                    bot.body.apply_force_at_local_point((random.uniform(-forceMagnitude, forceMagnitude), random.uniform(-forceMagnitude, forceMagnitude)), (0,0))
                    # Print statement for debugging
                    print("angvel of bot "+str(bot)+" is "+str(bot.body.angular_velocity))
            counter = 0
        else:
            counter += 1

        # Update physics
        space.step(dt)

        # Necessary for drawing shapes
        space.debug_draw(draw_options)
        
        # Update screen image
        pygame.display.flip()

        # Increment clock
        clock.tick(dt)
        # Print statement for debugging
        print("time: "+str(pygame.time.get_ticks()))

        # Display frames/second as the display caption
        pygame.display.set_caption("fps: " + str(clock.get_fps()))