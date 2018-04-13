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
        self.move = 1 #assume initial velocity
        #get dimensions of screen
        w, h = pygame.display.get_surface().get_size()
        self.x0pos = w / 2 - width / 2
        self.y0pos = h / 2 - height / 2
        self._width = width #do not change
        self._height = height #do not change
        self._color = color
        self.wall_hit = 0
        #This can be expanded/changed without affecting overall logic if robot shape changes
    def draw(self):
        BLACK = (  0,   0,   0) #temp, adjust so you can change color based instantation
        pygame.draw.rect(self._screen, self._color, [self.x0pos, self.y0pos, self._width, self._height], 0)
    #define movement based on WeaselBot algorithm
    # def update(self):
    #     if not self.wall_hit
    #         self._movement();
    #     else
    #         self._wallcollide();
    #Define movement based on WeaselBot algorithm
    def _movement(self):
        self.x0pos = self.move + self.x0pos
        self.y0pos = self.move + self.y0pos


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
        #Check if collision will occur

        #update screen
        robot1._movement()


        screen.blit(background, (0, 0))
        #allsprites.draw(screen)
        robot1.draw()
        pygame.display.flip()




    pygame.quit()

if __name__ == '__main__':
        main()



#Then get robot building
#Get robot moving in a straight line
#Handle wall collisions
