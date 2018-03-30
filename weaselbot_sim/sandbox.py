#Calls the pygame library
import pygame
#Optional: Sets up constants and functions in global namespace
from pygame.locals import *
#
#
#
# Functions
def collision_prop():

#
# classes, may add if needed
# NOT DEFINED VARIABLES -- pseudocode to provide me variables to play with
class WeaselBot(self, screen, x, y, length, hue):
    #First thing first is to initialize constructor
    #Want to get color, shape, movement speed
    def __init__(self):
        self.color = pygame.color(hue) #not done, figure out once in WiFi
        self.robot = rect(screen, self.color, Rect, width=2) #Bad line, edit
        self.move = 5 #assume initial velocity
        #This can be expanded/changed without affecting overall logic if robot shape changes
        
    #Define movement based on WeaselBot algorithm
    def movement():
        #algorithm for movement


def main():
    pygame.init()
    screen = pygame.display.set_mode(500, 500)
    pygame.mouse.set_visible(0)
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250, 250, 250))
    #Display the Background While Setup Finishes
    screen.blit(background, (0,0))
    pygame.display.flip()
    #create objects
    robot1 = WeaselBot(self, screen, 100, 100)
    #create game clock
    clock = pygame.time.Clock()
    while 1:





    pygame.quit()

if __name__ == '__main__':
        main()
