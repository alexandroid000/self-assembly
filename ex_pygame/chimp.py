# Reader: First off, this code is basically just cliffnotes of a much more detailed
#account at https://www.pygame.org/docs/tut/ChimpLineByLine.html . I highly recommend
#reading that. I also assume that you're using this doc as help for understanding pygame
#and I hope I do that well. However, if you have a strong background in Python, a lot of notes
#may be pointless. I also recommend doing this all yourself. You could easily type the code out
# all yourself and get it working in an hour but that misses the point of the tutorial.
#
# Helpful notes:
# self in Python = this in C -> necessary because Python does not explicitly refer to the object in its constructors and such
# __init__ = Constructor -> closely tied to the self concept
# I come from C and C++ and thought this was dumb, so here's an article on why they're necessary
# http://neopythonic.blogspot.com/2008/10/why-explicit-self-has-to-stay.html
#
#
#
#modules: os is a package that handles paths in a plethora of operating systems, sys is miscellaneous
import os, sys
import pygame
#pygame.locals are commonly used constants and functions useful to have in global namespace
from pygame.locals import *
#prints warning message if font or mixer (sound) modules are not available
if not pygame.font: print 'Warning, fonts disabled'
if not pygame.mixer: print 'Warning, sound disabled'
#
#
#
#LOADING RESOURCES -- next two functions
#function loads an image, in the case below chimp.bmp and fist.bmp, and converts it into rect objects
def load_image(name, colorkey=None):
    #Creates a full path for name to the data subdirectory
    #Think linux: /home/user/directory/subdirectory/etc./data
    fullname = os.path.join('data', name)
    #try function attempts to load image to see if that works
    try:
        image = pygame.image.load(fullname)
    #If try function fails, exits system
    except pygame.error, message:
        print 'Cannot load image:', name
        raise SystemExit, message
    #Makes a new copy of a Surface and converts color format/depth for display
    image = image.convert()
    #Sets colorkey for image
    if colorkey is not None:
    #If colorkey is -1, color of image is set to topleft pixel color
        if colorkey is -1:
            colorkey = image.get_at((0,0))
        image.set_colorkey(colorkey, RLEACCEL)
    return image, image.get_rect()

#Function loads a sound file that takes name (of sound class) as an argument
def load_sound(name):
    #Dummy class that allows game to run without any extra error checking
    class NoneSound:
        def play(self): pass
    #checks if pygame.mixer was imported, returns dummy class if not
    if not pygame.mixer:
        return NoneSound()
    #creates a full path for name to data
    fullname = os.path.join('data', name)
    #does the same thing as the try/except function in load_image
    try:
        sound = pygame.mixer.Sound(fullname)
    except pygame.error, message:
        print 'Cannot load sound:', wav
        raise SystemExit, message
    #returns loaded sound object (by loaded, we mean it has path to data)
    return sound
#
#
#
# GAME OBJECT CLASSES
class Fist(pygame.sprite.Sprite):
    """moves a clenched fist on the screen, following the mouse"""
    #constructor -- image and rect objects included in Sprite (read docs)
    def __init__(self):
        pygame.sprite.Sprite.__init__(self) #call Sprite as base class
        self.image, self.rect = load_image('fist.bmp', -1)
        self.punching = 0   #punching state is true/false, calls function below
    #code that moves and updates the variables for the sprite (typically once per frame)
    def update(self):
        "move the fist based on the mouse position"
        pos = pygame.mouse.get_pos()
        self.rect.midtop = pos
        if self.punching:
            self.rect.move_ip(5, 10)
    #function does heart of game -- punching, which is just a true/false state
    def punch(self, target):
        "returns true if the fist collides with the target"
        if not self.punching:
            self.punching = 1
            hitbox = self.rect.inflate(-5, -5) #inflates fist rect at point and saves that to hitbox
            return hitbox.colliderect(target.rect)
    #resets punching function
    def unpunch(self):
        "called to pull the fist back"
        self.punching = 0

class Chimp(pygame.sprite.Sprite):
    """ moves a monkey critter across the screen. It can spin the monkey when it is punched."""
    #constructor
    def __init__(self):
        pygame.sprite.Sprite.__init__(self) #call Sprite initializer
        self.image, self.rect = load_image('chimp.bmp', -1) #calls base objects
        screen = pygame.display.get_surface() #sets screen to surface
        self.area = screen.get_rect() #gets area of screen
        self.rect.topleft = 10, 10    #
        self.move = 9
        self.dizzy = 0

    def update(self):
        "walk or spin, depending on the monkeys state"
        #true/false statements, pretty clear, updates once per frame
        if self.dizzy:
            self._spin()
        else:
            self._walk()
    #underscore = convention that marks the functions to only be used by Chimp class (but not actually private)
    def _walk(self):
        "move the monkey across the screen, and turn at the ends"
        newpos = self.rect.move((self.move, 0))
        if self.rect.left < self.area.left or \
            self.rect.right > self.area.right:
            self.move = -self.move
            newpos = self.rect.move((self.move, 0))
            self.image = pygame.transform.flip(self.image, 1, 0)
        self.rect = newpos

    def _spin(self):
        "spin the monkey image"
        center = self.rect.center
        #Use increments of 12 cuz 360 % 12 = 0. Seamless transition back to original form
        self.dizzy += 12
        if self.dizzy >= 360:
            self.dizzy = 0
            self.image = self.original
        #utilizes rotate function of transform library
        else:
            rotate = pygame.transform.rotate
            self.image = rotate(self.original, self.dizzy)
        self.rect = self.image.get_rect(center=center)
    #Sets dizzy to true if not already
    def punched(self):
        "this will cause the monkey to start to spinning"
        if not self.dizzy:
            self.dizzy = 1
            self.original = self.image
#
#
# MAIN FUNCTION
def main():
    #initializes everything; creates a small black window with details filled in later in code
    pygame.init()   #Goes through and initializes imported pygame modules
    screen = pygame.display.set_mode((468, 60)) #creates your surface to the size you want
    pygame.display.set_caption('Monkey Fever') #sets title of GUI
    pygame.mouse.set_visible(0)
    #Fills in the background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250, 250, 250))    #fills background with white
    #Put text on background, centered
    if pygame.font:
        font = pygame.font.Font(None, 36)
        text = font.render("Pummel The Chimp and Win $$$", 1, (10, 10, 10))
        textpos = text.get_rect(centerx=background.get_width()/2)
        background.blit(text, textpos)
    #Display the Background While Setup Finishes
    screen.blit(background, (0,0))
    pygame.display.flip()
    #Prepare Game Object
    whiff_sound = load_sound('whiff.wav')
    punch_sound = load_sound('punch.wav')
    chimp = Chimp()
    fist = Fist()
    allsprites = pygame.sprite.RenderPlain((fist, chimp))
    clock = pygame.time.Clock()
    #Main Loop --- continuous
    while 1:
        clock.tick(60)
        #Handle Input Events
        for event in pygame.event.get():
            if event.type == QUIT:
                going = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                going = False
            elif event.type == MOUSEBUTTONDOWN:
                if fist.punch(chimp):
                    punch_sound.play()
                    chimp.punched()
                else:
                    whiff_sound.play()
            elif event.type == MOUSEBUTTONUP:
                fist.unpunch()

        allsprites.update()

        screen.blit(background, (0, 0))
        allsprites.draw(screen)
        pygame.display.flip()

    pygame.quit()

if __name__ == '__main__':
    main()
