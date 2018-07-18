'''
This pygame simulation incorporates the pymunk library, which uses the Chipmunk
physics engine. However, adding joints after initial start-up creates extreme
behavior that I have had trouble debugging. Configurations made in the initialization
of the simulation do not have this extreme behavior.

To run, type: "python pymunk-simulation.py m n" where m = number of units, n = sides per unit
    Note: I recommend "python pymunk-simulation.py 64 6"
To quit, press "Esc"
To print to .png, press "p"
To change speed, press "1-9" (default=5)
To pause, press "Space"

TODO:
General:
- Fix time counting
- Fix force application frequency
Option 1: 
- Fix extreme behavior
Option 2:
- Automate joint creation
- Simplify shape creation
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
proximityMargin = 2                 # extra margin for imperfectly-aligned shapes to connect (default=2)
(screenw, screenh) = (400, 500)     # screen dimensions (default=(400, 500))
dampingConstant = 0.99996           # Amount of velocity preserved per second (default=0.99996 = 0.004% loss per second)
mass = 100                          # weaselball mass (default=100)
r = 15                              # shortest radius from center to each side (default=15)
elasticityConstant = 1              # weaselball elasticity (default=0)
frictionConstant = 0.9              # weaselball friction (default=0.9)
dt = 60                             # clock speed multiplier (default=60)
forceMagnitude = 0.0005             # magnitude of min and max range of random linear force applied to weaselballs (default=0.0005)
forcefreq = 50                      # frequency at which linear force is applied (default=50)

# Returns False if in same group
def groupCheck(arbiter, space, data):
    bot1, bot2 = arbiter.shapes
    if bot1.group == bot2.group:
        return False
    else:
        return True

# Collision callback function to constrain shapes
def addJoints(arbiter, space, data):
    # If shapes are within proximity and colliding at 2
    # points, add a PivotJoint to each point of contact
    maxProximityValue = (2*math.cos(angle/2)*r)**2 + (math.sin(angle/2)*r)**2
    minProximityValue = (2*math.cos(angle/2)*r)**2
    bot1, bot2 = arbiter.shapes
    if ((bot1.body.position.x-bot2.body.position.x)**2 + (bot1.body.position.y-bot2.body.position.y)**2) < maxProximityValue + proximityMargin:
        if ((bot1.body.position.x-bot2.body.position.x)**2 + (bot1.body.position.y-bot2.body.position.y)**2) >= minProximityValue:
            if len(arbiter.contact_point_set.points) == 2:
                space.add(pymunk.PivotJoint(bot1.body, bot2.body, arbiter.contact_point_set.points[0].point_a))
                space.add(pymunk.GearJoint(bot1.body, bot2.body, bot2.body.angle - bot1.body.angle, 1))
                print("joint: "+str(bot1)+', '+str(bot2))

                # Make all units in new configuration the same group
                groupToChange = bot2.group
                for bot in space.shapes:
                    if isinstance(bot, pymunk.Poly) and bot.body != None:
                        if bot.group == groupToChange:
                            bot.group = bot1.group

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

    '''# Option 1: Randomized balls that connect
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
        shape.group = i
        space.add(body, shape)
    # End of Option 1
    '''

    # Option 2: Given arrangement of balls
    # Instantiate weaselball surface shapes and physics bodies
    # with established x- and y-positions
    xylist = [[screenw/2-r,screenh/2-r], [screenw/2+r, screenh/2-r], [screenw/2-r, screenh/2+r], [screenw/2+r, screenh/2+r]]
    jointlist = [[screenw/2,screenh/2-r], [screenw/2-r,screenh/2], [screenw/2+r,screenh/2], [screenw/2,screenh/2+r]]
    for i in range(4):
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255), 1)
        inertia = pymunk.moment_for_poly(mass, vertices)
        body = pymunk.Body(mass, inertia)
        body.position = xylist[i]
        body.velocity = (0,0)
        shape = pymunk.Poly(body, vertices)
        shape.collision_type = COLLTYPE_UNIT
        shape.elasticity = elasticityConstant
        shape.friction = frictionConstant
        shape.color = color
        shape.group = 0
        space.add(body, shape)
    # Add joints
    botlist = []
    for bot in space.shapes:
        if isinstance(bot, pymunk.Poly) and bot.body != None:
            botlist.append(bot.body)
    space.add(pymunk.PivotJoint(botlist[0], botlist[1], jointlist[0]))
    space.add(pymunk.GearJoint(botlist[0], botlist[1], 0, 1))
    space.add(pymunk.PivotJoint(botlist[0], botlist[2], jointlist[1]))
    space.add(pymunk.GearJoint(botlist[0], botlist[2], 0, 1))
    space.add(pymunk.PivotJoint(botlist[1], botlist[3], jointlist[2]))
    space.add(pymunk.GearJoint(botlist[1], botlist[3], 0, 1))
    space.add(pymunk.PivotJoint(botlist[2], botlist[3], jointlist[3]))
    space.add(pymunk.GearJoint(botlist[2], botlist[3], 0, 1))
    # End of Option 2

    # Instantiate collision handler
    ch = space.add_collision_handler(COLLTYPE_UNIT, COLLTYPE_UNIT)
    ch.data["surface"] = screen
    ch.pre_solve = groupCheck
    ch.post_solve = addJoints

    # Counter for random motion
    counter = 0

    # Total time
    TotalTime = 0

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
                dt = 400
            elif event.type == KEYDOWN and event.key == K_9:
                dt = 800

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

        # Print impulse of each pinjoint - debugging
        if dt > 0:
            a = 0
            maxhit = False
            for joint in space.constraints:
                if isinstance(joint, pymunk.PivotJoint):
                    if joint.impulse/dt > 0.003:
                        print("Joint "+str(a)+" force is: "+str(joint.impulse/dt))
                        a += 1
                        if joint.impulse/dt > 0.01:
                            maxhit = True
        if maxhit:
            dt = 1

        # Update physics
        space.step(dt)

        # Necessary for drawing shapes
        space.debug_draw(draw_options)

        # Update total time
        TotalTime += dt

        # Instructions text
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'Esc\' to quit', False, BLACK), (0,0))
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'p\' to print to .png', False, BLACK), (0,10))
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'1-9\' to change fps (default=5)', False, BLACK), (0,20))
        screen.blit(pygame.font.SysFont('arial', 10).render('Press \'Space\' to pause', False, BLACK), (0,30))
        screen.blit(pygame.font.SysFont('arial', 10).render("Total time: "+str(TotalTime), False, BLACK), (0,screenh-10))
        
        # Update screen image
        pygame.display.flip()

        # Increment clock
        clock.tick()

        # Display frames/second as the display caption
        pygame.display.set_caption(str(sides)+"-Sided Weaselball Simulation")