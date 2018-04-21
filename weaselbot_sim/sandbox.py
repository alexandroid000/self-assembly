#Calls the pygame library
import pygame
import random
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
#calculates the likelihood of a collision occuring
#def collision():
    #Case 1: Hitting another WeaselBot



class WeaselBot():
    def __init__(self, screen, width, height, xspeed, yspeed, color):
        self.movex = xspeed #assume initial velocity
        self.movey = yspeed
        #get dimensions of screen
        self.screen = screen
        self.wscreen, self.hscreen = pygame.display.get_surface().get_size()
        self.x0pos = random.randint(0, self.wscreen-width)
        self.y0pos = random.randint(0, self.hscreen-height)
        self._width = width #do not change
        self._height = height #do not change
        self._color = color
        self.collision = False

    def draw(self, screen):
        BLACK = (  0,   0,   0) #temp, adjust so you can change color based instantation
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

#merges two objects into one
def collision_merge(collisions):
    #create a new, merged object
    #delete the previous two objects
    bot1 = collisions[0]
    bot2 = collisions[1]
    width = bot1._width + bot2._width
    height = bot1._height + bot2._height
    #this is temporary before algorithm gets developed
    xspeed = bot1.movex
    yspeed = bot2.movey
    mergedbot = WeaselBot(bot1.screen, width, height, xspeed, yspeed, BLUE)

    mergedbot.x0pos = bot1.x0pos
    mergedbot.y0pos = bot1.y0pos
    if(mergedbot.x0pos > bot2.x0pos):
        mergedbot.x0pos = bot2.x0pos
    if(mergedbot.y0pos > bot2.y0pos):
        mergedbot.y0pos = bot2.y0pos
    return mergedbot

class WeaselBotsGroup():
    def __init__(self):
        self.listofweasels = []
    def add(self, bot):
        self.listofweasels.append(bot)
    #handle collision here
    def update(self):
        collisions = []
        for i in self.listofweasels:
            #makes a list of collisions
            if i.collision:
                collisions.append(i)
            #otherwise, procede normally
            else:
                i.movement()
        #checks for collisions
        for i in self.listofweasels:
            for j in self.listofweasels:
                #makes sure they're not the same object and if collision has previously occured
                if i == j or (i.collision == True and j.collision == True):
                    break;
                #sees if x,y positions match
                for jxpos in range(j.x0pos, j.x0pos+j._width):
                    if i.x0pos <= jxpos <= i.x0pos+i._width:
                        for jypos in range(j.y0pos, j.y0pos+j._height):
                            if i.y0pos <= jypos <= i.y0pos+i._height:
                                j.collision = True
                                i.collision = True
        #if collisions occur, make a new weaselbot object and add it to the list
        if collisions:
            mergedbot = collision_merge(collisions)
            self.listofweasels.remove(collisions[0])
            self.listofweasels.remove(collisions[1])
            del collisions
            self.listofweasels.append(mergedbot)

            #call collision merge on objects

    def draw(self, screen):
        for i in self.listofweasels:
            i.draw(screen)

def main():
    pygame.init()
    screen = pygame.display.set_mode((400, 500))
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
    weaselbots = WeaselBotsGroup();
    #I dislike how clunky this is but I can't get around it right now
    weaselbots.add(robot1)
    weaselbots.add(robot2)
    weaselbots.add(robot3)
    #create game clock
    clock = pygame.time.Clock()
    #Game loop
    while 1:
        #increment clock
        clock.tick(60)
        #update screen
        weaselbots.update()
        #Check if collision will occur
        #updates screen image (redraws screen everytime)
        screen.blit(background, (0, 0))
        weaselbots.draw(screen)
        pygame.display.flip()


    pygame.quit()

if __name__ == '__main__':
        main()



#Then get robot building
#Get robot moving in a straight line
#Handle wall collisions
