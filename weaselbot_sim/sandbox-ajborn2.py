'''
This file is essentially outdated as sandbox2-ajborn2.py
shows much more promise.

To run, type "python sandbox-ajborn2.py n" where n = number of units
Press "q" to quit simulation

TODO:
- Adjusting to proper connect points => collision occurs only at certain angles
- Deal with random motion
    - New constraints on walls
- Deal with rotational motion(pygame.transform)
    - Collision detection with rotated shapes
- Handle 6- and 8-sided shapes.
- Other noted occurrences:
    - For >200 units, configurations sometimes pass display boundaries
    - Overlap issues
'''

# Import libraries
import pygame
import random
import keyboard
import sys
import math

# Optional: Sets up constants and functions in global namespace
# TODO: necessary?
from pygame.locals import *

# Declare global colors
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)

# Define Configuration class
class Configuration():
    # Instantiation function
    def __init__(self, weaselbots, xcenter, ycenter, theta, xspeed, yspeed, thetaspeed):
        self.weaselbots = weaselbots
        self.xcenter = xcenter
        self.ycenter = ycenter
        self.theta = theta
        self.xspeed = xspeed
        self.yspeed = yspeed
        self.thetaspeed = thetaspeed

    # Collision function to check against the nth configuration
    def collision(self, configlist, n):
        otherConfig = configlist[n]
        thisCnt = len(self.weaselbots)
        otherCnt = 0

        # Check every potential bot-to-bot collision
        for bot1 in self.weaselbots:
            for bot2 in otherConfig.weaselbots:
                # If collision, adjust position for each box relative to configuration
                if math.sqrt((bot1.xpos - bot2.xpos)**2 + (bot1.ypos - bot2.ypos)**2) <= (bot1.r + bot2.r):
                    for bot in otherConfig.weaselbots:
                        self.weaselbots.append(bot)
                        otherCnt += 1
                    # Recalculate center of configuration
                    self.xcenter = 0
                    self.ycenter = 0
                    for bot in self.weaselbots:
                        self.xcenter += bot.xpos
                        self.ycenter += bot.ypos
                    self.xcenter = int(self.xcenter/(thisCnt + otherCnt))
                    self.ycenter = int(self.ycenter/(thisCnt + otherCnt))
                    # Recalculate configuration speed
                    # TODO Adjust for rotational motion
                    self.xspeed = (self.xspeed*thisCnt + otherConfig.xspeed*otherCnt)/(thisCnt + otherCnt)
                    self.yspeed = (self.yspeed*thisCnt + otherConfig.yspeed*otherCnt)/(thisCnt + otherCnt)
                    return True
        return False

    # Drawing function
    def draw(self, screen):
        for bot in self.weaselbots:
            pygame.draw.rect(screen, bot.color, [bot.xpos - bot.r, bot.ypos - bot.r, 2*bot.r, 2*bot.r], 0)

    # Movement function to calculate new location
    def move(self, screenWidth, screenHeight):
        self.xcenter += self.xspeed
        self.ycenter += self.yspeed
        self.theta += self.thetaspeed
        for bot in self.weaselbots:
            # TODO Adjust for rotational motion
            bot.xpos += self.xspeed
            bot.ypos += self.yspeed
        # Handle edge bounces
        for bot in self.weaselbots:
            breakflag = False
            if (bot.xpos - bot.r <= 0) or (bot.xpos + bot.r >= screenWidth):
                self.xspeed *= -1
                breakflag = True
            if (bot.ypos - bot.r <= 0) or (bot.ypos + bot.r >= screenHeight):
                self.yspeed *= -1
                breakflag = True
            if breakflag:
                break

# Define WeaselBot class
class WeaselBot():
    def __init__(self, r, color, xpos, ypos):
        self.r = r
        self.color = color
        self.xpos = xpos
        self.ypos = ypos   


# Main function body (sys.argv[1] is an integer n in the executable "py PolyominoGenerator.py n")
if __name__ == '__main__':
    # Initialize game
    pygame.init()

    # Set display size
    (screenw, screenh) = (400, 500)
    screen = pygame.display.set_mode((screenw, screenh))

    # Set background
    background = pygame.Surface(screen.get_size()).convert()
    background.fill(GRAY)

    # Display the background
    screen.blit(background, (0,0))
    pygame.display.flip()

    # Initialize configurations
    r = 10
    weaselballNum = int(sys.argv[1])
    splitNum = math.ceil(math.sqrt(weaselballNum))
    splitLenw = int(screenw/splitNum)
    splitLenh = int(screenh/splitNum)
    configlist = []
    for i in range(weaselballNum):
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        randx = random.randint(int(i%splitNum)*splitLenw+r, (int(i%splitNum)+1)*splitLenw-r)
        randy = random.randint(math.floor(i/splitNum)*splitLenh+r, (math.floor(i/splitNum)+1)*splitLenh-r)
        weasel = WeaselBot(r, color, randx, randy)
        globals()['config%s' % i] = Configuration([weasel], weasel.xpos, weasel.ypos, 0, random.randint(-3, 3), random.randint(-3, 3), 0)
        configlist.append(globals()['config%s' % i])


    # Create game clock
    clock = pygame.time.Clock()

    # Game loop
    while True:
        # Increment clock
        clock.tick(60)

        # Update screen
        for config in configlist:
            if config is not None:
                config.move(screenw, screenh)

        # Check if collision will occur
        for i in range(len(configlist)):
            if configlist[i] is not None:
                for j in range(i+1, len(configlist)):
                    if configlist[j] is not None:
                        if configlist[i].collision(configlist, j):
                            configlist[j] = None

        # Update screen image
        screen.blit(background, (0, 0))
        for config in configlist:
            if config is not None:
                config.draw(screen)
        pygame.display.flip()

        # For events, do nothing
        for event in pygame.event.get():
            None

        # If "q" pressed, exit game
        if keyboard.is_pressed('q'):
            pygame.quit()

