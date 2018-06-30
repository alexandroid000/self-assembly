'''
This pygame simulation incorporates the pymunk library, which uses the Chipmunk
physics engine. However, the joint connections and extreme angular velocities 
have been difficult to deal with so far.

To run, type: "python pymunk-simulation.py m n" where m = number of units, n = sides per unit
    Note: I recommend "python pymunk-simulation.py 64 6"
To quit, press "Esc"
To print to .png, press "p"
To change speed, press "1-9" (default=5)
To pause, press "Space"

TODO:
- Deal with physics, erm, difficulties
    - Currently using PivotJoint for connections, but with the randomly-added forces, 
        this can lead to extreme angular velocities
- Play with type of joint
- Fix the way random forces are applied
'''

__version__ = "$Id:$"
__docformat__ = "reStructuredText"

# Python library import
import pygame
import random
import math
import sys
import time
from pygame.locals import *
#from pygame.color import *
import pymunk
import pymunk.pygame_util
import os

# Declare global colors
# COLOR = (R, G, B) => 0-255 for each
BLACK = (0,0,0)
GRAY = (200, 200, 200)   

# Global constants
COLLTYPE_UNIT = 1                   # Collision type for weaselballs
proximityMargin = 2                 # extra margin for imperfectly-aligned shapes to connect
(screenw, screenh) = (400, 500)     # screen dimensions
dampingConstant = 0.99996           # Amount of velocity preserved per second (0.99996 = 0.004% loss per second)
mass = 100                          # weaselball mass
r = 15                              # weaselball radius from center to each vertex
elasticityConstant = 0              # weaselball elasticity
frictionConstant = 0.9              # weaselball friction
dt = 60                             # clock speed multiplier (default=60)
forceMagnitude = 0.0005             # magnitude of min and max range of random linear force applied to weaselballs
forcefreq = 50                      # frequency at which linear force is applied

# Collision callback function to constrain shapes
def addJoints(arbiter, space, data):
    # If shapes are within proximity and colliding at 2
    # points, add a PivotJoint to each point of contact
    maxProximityValue = (2*math.cos(angle/2)*r)**2 + (math.sin(angle/2)*r)**2
    minProximityValue = (2*math.cos(angle/2)*r)**2
    bot1, bot2 = arbiter.shapes
    if (bot1, bot2) not in joints:
        joints[(bot1, bot2)] = False
        joints[(bot2, bot1)] = False
    if joints[(bot1, bot2)] == False:
        if ((bot1.body.position.x-bot2.body.position.x)**2 + (bot1.body.position.y-bot2.body.position.y)**2) < maxProximityValue + proximityMargin:
            if ((bot1.body.position.x-bot2.body.position.x)**2 + (bot1.body.position.y-bot2.body.position.y)**2) > minProximityValue + proximityMargin:
                #space.add(pymunk.PinJoint(bot1.body, bot2.body))
                if len(arbiter.contact_point_set.points) == 2:
                    for c in arbiter.contact_point_set.points:
                        space.add(pymunk.PivotJoint(bot1.body, bot2.body, c.point_a))
                    joints[(bot1, bot2)] = True
                    joints[(bot2, bot1)] = True

# Main function body
if __name__ == '__main__':

    # Initialize game
    pygame.init()
    pygame.font.init()
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

    # Total time
    time = 0

    # Game loop
    while running:

        # Handle external events (key presses)
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                running = False
            elif event.type == KEYDOWN and event.key == K_p:
                mypath = str(os.getcwd())+"\simulation_images"
                if not os.path.exists(mypath):
                    os.mkdir(mypath)
                localtime = str(time.strftime("%Y-%m-%d-%H-%M-%S."+str(pygame.time.get_ticks()%1000), time.localtime()))
                pygame.image.save(screen, mypath+"\weaselball_sim_"+localtime+".png")
            elif event.type == KEYDOWN and event.key == K_SPACE:
                dt = 0
            elif event.type == KEYDOWN and event.key == K_1:
                dt = 1
            elif event.type == KEYDOWN and event.key == K_2:
                dt = 4
            elif event.type == KEYDOWN and event.key == K_3:
                dt = 10
            elif event.type == KEYDOWN and event.key == K_4:
                dt = 30
            elif event.type == KEYDOWN and event.key == K_5:
                dt = 60
            elif event.type == KEYDOWN and event.key == K_6:
                dt = 100
            elif event.type == KEYDOWN and event.key == K_7:
                dt = 200
            elif event.type == KEYDOWN and event.key == K_8:
                dt = 300
            elif event.type == KEYDOWN and event.key == K_9:
                dt = 400

        # Clear screen
        screen.fill(GRAY)        

        # Randomized motion by applying a random force on each unit's center
        if counter == forcefreq:
            for bot in space.shapes:
                if isinstance(bot, pymunk.Poly) and bot.body != None:
                    bot.body.apply_force_at_local_point((random.uniform(-forceMagnitude, forceMagnitude), random.uniform(-forceMagnitude, forceMagnitude)), (0,0))
            counter = 0
        else:
            counter += 1

        # Update physics
        space.step(dt)

        # Necessary for drawing shapes
        space.debug_draw(draw_options)

        # Update total time
        time += dt

        # Instructions text
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'Esc\' to quit', False, BLACK), (0,0))
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'p\' to print to .png', False, BLACK), (0,10))
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'1-9\' to change fps (default=5)', False, BLACK), (0,20))
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'Space\' to pause', False, BLACK), (0,30))
        screen.blit(pygame.font.SysFont('arial', 10).render("Total time: "+str(time), False, BLACK), (0,screenh-10))
        
        # Update screen image
        pygame.display.flip()

        # Increment clock
        clock.tick()

        # Display frames/second as the display caption
        pygame.display.set_caption(str(sides)+"-Sided Weaselball Simulation")