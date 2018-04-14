#Calls the pygame library
import pygame
#Optional: Sets up constants and functions in global namespace
from pygame.locals import *
#Declare global colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
#
#
# Functions
#calculates the likelihood of a collision occuring
#def collision_prop():
    #Case 1: Hitting another WeaselBot
    #case 2: Hitting a wall

#merges two objects into one
#def collision_merge(WeaselBot one, WeaselBot two):
    #create a new, merged object
    #delete the previous two objects
# classes, may add if needed
class WeaselBot(pygame.sprite.Sprite):
    def __init__(self, screen, width, height, color):
        self._screen = screen
        self.movex = 1 #assume initial velocity
        self.movey = 0
        #get dimensions of screen
        w, h = pygame.display.get_surface().get_size()
        self.x0pos = 150
        self.y0pos = 150
        self._width = width #do not change
        self._height = height #do not change
        self._color = color
        self.collision = 0

    def draw(self):
        BLACK = (  0,   0,   0) #temp, adjust so you can change color based instantation
        pygame.draw.rect(self._screen, self._color, [self.x0pos, self.y0pos, self._width, self._height], 0)
    #define movement based on WeaselBot algorithm
    def _movement(self):
        #the if conditions shouldn't change as they determine a wall hit (robot is 20x20 and screen is 500x500)
        self.x0pos = self.movex + self.x0pos
        self.y0pos = self.movey + self.y0pos
        if(self.x0pos < 0 or self.x0pos > 480):
            self.movex = -self.movex
        if(self.y0pos < 0 or self.y0pos > 480):
            self.movey = -self.movey



    # def _wallcollide(self):
    #     #What happens when it hits the walls

def main():
    pygame.init()
    screen = pygame.display.set_mode((500, 500))
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
    robot1 = WeaselBot(screen, width, height, BLACK)
    #allsprites = pygame.sprite.RenderPlain((robot1))
    #create game clock
    clock = pygame.time.Clock()
    while 1:
        #increment clock
        clock.tick(60)
        #update screen
        robot1._movement()
        #Check if collision will occur
        #updates screen image (redraws screen everytime)
        screen.blit(background, (0, 0))
        robot1.draw()
        pygame.display.flip()




    pygame.quit()

if __name__ == '__main__':
        main()



#Then get robot building
#Get robot moving in a straight line
#Handle wall collisions
