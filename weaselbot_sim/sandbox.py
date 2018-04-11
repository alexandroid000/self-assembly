#Calls the pygame library
import pygame
#Optional: Sets up constants and functions in global namespace
from pygame.locals import *
#
#
#
# Functions
#calculates the likelihood of a collision occuring
def collision_prop():
    #Case 1: Hitting another WeaselBot
    #case 2: Hitting a wall

#merges two objects into one
def collision_merge(WeaselBot one, WeaselBot two):
    #create a new, merged object
    #delete the previous two objects

# classes, may add if needed
# NOT DEFINED VARIABLES -- pseudocode to provide me variables to play with
class WeaselBot(self, screen, x, y, length, hue):
    #First thing first is to initialize constructor
    #Want to get color, shape, movement speed
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.color = pygame.color(hue) #not done, figure out once in WiFi
        self.robot = pygame.draw.rect(screen, self.color, Rect, width=2)
        self.move = 5 #assume initial velocity
        #self.collided = 0 #state of collision
        self.wall_hit = 0
        #This can be expanded/changed without affecting overall logic if robot shape changes
    #define movement based on WeaselBot algorithm
    def update(self)
        if not self.wall_hit
            self._movement();
        else
            self._wallcollide();


    #Define movement based on WeaselBot algorithm
    def _movement(self):
        #algorithm for movement
        # this is wrong but it's a good sample
        newpos = self.robot.move((self.move, 0))
        if self.rect.left < self.area.left or \
            self.rect.right > self.area.right:
            self.move = -self.move
            newpos = self.rect.move((self.move, 0))
            self.image = pygame.transform.flip(self.image, 1, 0)
        self.rect = newpos

    def _wallcollide(self):
        #What happens when it hits the walls

def main():
    pygame.init()
    screen = pygame.display.set_mode(500, 500)
    pygame.mouse.set_visible(0)
    #fills in background
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
        #increment clock
        clock.tick(60)
        #increment _movement

        #LATER -- check if collision occurs

        #update screen
        allsprites.update()

        screen.blit(background, (0, 0))
        allsprites.draw(screen)
        pygame.display.flip()




    pygame.quit()

if __name__ == '__main__':
        main()
