'''
This file is Austin Born's (ajborn2@illinois.edu) recreation
of the sandbox pygame simulation file. Adjustments made here
are not necessarily reflective of the current state of the 
simulation, but different changes are tested here.

To run, type "python sandbox-ajborn2.py n" where n = number of units
Press "q" to quit simulation

TODO:
- Specifics of collisions (overlap issues, adjusting to proper connect points => collision occurs at )
- Deal with random motion 
- Deal with rotational motion(pygame.transform)
    - Rotational center of mass
    - Collision detection with rotated shapes
- Handle 6- and 8-sided shapes.
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
    def __init__(self, weaselbots, xmin, xmax, ymin, ymax, xspeed, yspeed):
        self.weaselbots = weaselbots
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.xspeed = xspeed
        self.yspeed = yspeed

    def collision(self, configlist, n):
        otherConfig = configlist[n]
        cnt = 0
        for bot1 in self.weaselbots:
            bot1Rect = pygame.Rect(bot1.xpos, bot1.ypos, bot1.width, bot1.height)
            for bot2 in otherConfig.weaselbots:
                bot2Rect = pygame.Rect(bot2.xpos, bot2.ypos, bot2.width, bot2.height)
                if bot1Rect.colliderect(bot2Rect):
                    for bot in otherConfig.weaselbots:
                        self.weaselbots.append(bot)
                        cnt += 1
                        if bot.xpos < self.xmin:
                            self.xmin = bot.xpos
                        elif bot.xpos+bot.width > self.xmax:
                            self.xmax = bot.xpos+bot.width
                        if bot.ypos < self.ymin:
                            self.ymin = bot.ypos
                        elif bot.ypos+bot.height > self.ymax:
                            self.ymax = bot.ypos+bot.height
                    self.xspeed = (self.xspeed*len(self.weaselbots) + otherConfig.xspeed*cnt)/(len(self.weaselbots) + cnt)
                    self.yspeed = (self.yspeed*len(self.weaselbots) + otherConfig.yspeed*cnt)/(len(self.weaselbots) + cnt)
                    return True
        return False


    def draw(self, screen):
        for weaselball in self.weaselbots:
            pygame.draw.rect(screen, weaselball.color, [weaselball.xpos, weaselball.ypos, weaselball.width, weaselball.height], 0)

    def move(self, screenWidth, screenHeight):
        self.xmin += self.xspeed
        self.xmax += self.xspeed
        self.ymin += self.yspeed
        self.ymax += self.yspeed
        for bot in self.weaselbots:
            bot.xpos += self.xspeed
            bot.ypos += self.yspeed
        if (self.xmin < 0) or (self.xmax > screenWidth):
            self.xspeed *= -1
        if (self.ymin < 0) or (self.ymax > screenHeight):
            self.yspeed *= -1

# Define WeaselBot class
class WeaselBot():
    def __init__(self, width, height, color, xpos, ypos):
        self.width = width
        self.height = height
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
    w, h = 20, 20
    weaselballNum = int(sys.argv[1])
    splitNum = math.ceil(math.sqrt(weaselballNum))
    splitLenw = int(screenw/splitNum)
    splitLenh = int(screenh/splitNum)
    configlist = []
    for i in range(weaselballNum):
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        randx = random.randint(int(i%splitNum)*splitLenw, (int(i%splitNum)+1)*splitLenw-w)
        randy = random.randint(math.floor(i/splitNum)*splitLenh, (math.floor(i/splitNum)+1)*splitLenh-h)
        weasel = WeaselBot(w, h, color, randx, randy)
        globals()['config%s' % i] = Configuration([weasel], weasel.xpos, weasel.xpos+weasel.width, weasel.ypos, weasel.ypos+weasel.height, random.randint(-3, 3), random.randint(-3, 3))
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

        for event in pygame.event.get():
            if event.type == MOUSEBUTTONUP:
               None

        # If "q" pressed, exit game
        if keyboard.is_pressed('q'):
            pygame.quit()

