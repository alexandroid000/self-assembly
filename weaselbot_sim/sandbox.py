#How it works:
# - Each weaselbot is randomly generated on the screen
# - When they collide, they don't actually form one entity; the collided objects speed parameters are made identical
# - Handling of all the collided objects is taken care of in the WeaselBotsGroup class


# Current Errors:
#  - On collision, the robots do not stick together like they should
#  - Fails at handling collisions between already collided objects and single objects
# - Do not know how it would handle simultaneous collision between 3+ objects
# - Gets stuck at the borders (this is probably due to a misplaced == sign when there should be a >= sign)
# - Can't handle non-square shapes at present (that was the change I tried to implement and then everything stopped working)




#Calls the pygame library
import pygame, random, math
from collections import defaultdict
#Optional: Sets up constants and functions in global namespace
from pygame.locals import *
#Declare global colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
#
#
# Functions
class WeaselBot():
    def __init__(self, screen, width, height, xspeed, yspeed, color):
        self.movex = xspeed #assume initial velocity
        self.movey = yspeed
        #get dimensions of screen
        self.wscreen, self.hscreen = pygame.display.get_surface().get_size()
        self.x0pos = random.randint(0, self.wscreen-width)
        self.y0pos = random.randint(0, self.hscreen-height)
        self._width = width #do not change
        self._height = height #do not change
        self._color = color
        self.collision = False

    def draw(self, screen):
        BLACK = (  0,   0,   0) #temp, adjust so you can change color based instantation
        #pygame.draw.polygon(screen, self._color, self.points, 0)
        pygame.draw.rect(screen, self._color, [self.x0pos, self.y0pos, self._width, self._height], 0)

    #define movement based on WeaselBot algorithm
    def movement(self):
        #insert WeaselBot algorithm here
        self.x0pos = self.movex + self.x0pos
        self.y0pos = self.movey + self.y0pos
        #the if conditions shouldn't change as they determine a wall hit (robot is 20x20 and screen is 500x500)
        if(self.x0pos < 0 or self.x0pos > (self.wscreen- self._width)):
            self.movex = -self.movex
        if(self.y0pos < 0 or self.y0pos > (self.hscreen - self._height)):
            self.movey = -self.movey

class WeaselBotsGroup():
    def __init__(self):
        self.listofweasels = []
    def add(self, bot):
        self.listofweasels.append(bot)
    #handle collision here
    def update(self):
        #Make a dictionary of lists
        collisions = {}
        for i in self.listofweasels:
            collisions[i] = []
            for j in self.listofweasels:
                #makes sure they're not the same object and if collision has previously occured
                if i == j or (i.collision == True and j.collision == True):
                    break;
                #sees if x,y positions match
                for jxpos in range(j.x0pos, j.x0pos+j._width):
                    if i.x0pos <= jxpos <= i.x0pos+i._width:
                        for jypos in range(j.y0pos, j.y0pos+j._height):
                            if i.y0pos <= jypos <= i.y0pos+i._height:
                                collisions[i].append(j)


        #if collisions occur, make a new weaselbot object and add it to the list
        self.collision_merge(collisions)
        del collisions

    def draw(self, screen):
        for i in self.listofweasels:
            i.draw(screen)

#each object acts identically so they appear (on the screen) to be one object
    def collision_merge(self, collisions):
        for i in collisions:
            if collisions[i] == []:
                i.movement()
            for j in collisions[i]:
                j.movex = i.movex
                j.movey = i.movey
                for k in collisions[j]:
                    if k == i:
                        collisions[j].remove(k)

def redefine_bots_borders(bot1, bot2):
    # All this is error checking to make sure it doesn't bounce outside screen
    # BROKEN --- NEEDS TO BE FIXED
    if(bot1.x0pos + bot1._width == bot2.x0pos):
        bot1.y0pos = bot2.y0pos
        bot2.wscreen = bot2.wscreen - bot2._width
    elif(bot2.x0pos + bot2._width == bot1.x0pos):
        bot2.y0pos = bot1.y0pos
        bot1.wscreen = bot1.wscreen - bot1._width
    elif(bot1.y0pos + bot1._height == bot2.y0pos):
        bot1.x0pos = bot2.x0pos
        bot2.hscreen = bot2.hscreen - bot2._height
    elif(bot2.y0pos + bot2._height == bot1.y0pos):
        bot2.x0pos = bot1.x0pos
        bot1.hscreen = bot1.hscreen - bot1._height

def main():
    pygame.init()
    screen = pygame.display.set_mode((700, 500))
    #fills in background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250, 250, 250))
    #Display the Background While Setup Finishes
    screen.blit(background, (0,0))
    pygame.display.flip()
    #create objects
    width = 20
    height = 20
    robot1 = WeaselBot(screen, width, height, 1, 1, GREEN)
    robot2 = WeaselBot(screen, width, height, 2, 2, BLACK)
    robot3 = WeaselBot(screen, width, height, 3, 3, RED)
    robot4 = WeaselBot(screen, width, height, 4, 4, BLUE)
    robot5 = WeaselBot(screen, width, height, 5, 5, RED)
    weaselbots = WeaselBotsGroup();
    #I dislike how clunky this is but I can't get around it right now
    weaselbots.add(robot1)
    weaselbots.add(robot2)
    weaselbots.add(robot3)
    weaselbots.add(robot4)
    weaselbots.add(robot5)
    #create game clock
    clock = pygame.time.Clock()
    #Game loop
    done = False
    while not done:
        #increment clock
        clock.tick(60)
        #update screen
        weaselbots.update()
        #Check if collision will occur
        #updates screen image (redraws screen everytime)
        screen.blit(background, (0, 0))
        weaselbots.draw(screen)
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                done = True
                break;

    pygame.quit()

if __name__ == '__main__':
        main()
