#Calls the pygame library
import pygame
import random
#Optional: Sets up constants and functions in global namespace
from pygame.locals import *
#Declare global colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
#
#
# Functions
#calculates the likelihood of a collision occuring
#def collision():
    #Case 1: Hitting another WeaselBot


#merges two objects into one
#def collision_merge(WeaselBot one, WeaselBot two):
    #create a new, merged object
    #delete the previous two objects
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
                if i == j or (i.collision == True and j.collision == True):
                    break;
                for jxpos in range(j.x0pos, j.x0pos+j._width):
                    if i.x0pos <= jxpos <= i.x0pos+i._width:
                        for jypos in range(j.y0pos, j.y0pos+j._height):
                            if i.y0pos <= jypos <= i.y0pos+i._height:
                                j.collision = True
                                i.collision = True
        if collisions:
            pass;

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
