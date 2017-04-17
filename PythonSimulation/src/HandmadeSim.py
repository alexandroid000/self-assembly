import numpy as np

class WeaselBall(object):
    """Creates a Weasal ball that will randomly move around in random
    direction (with a fixed distance per step)
    Will connect to other weaselball object
    nodetypes is a array of length 4 of 4 module objects
    initialLocation is for location, length 2 array
    orientation is direction ball is facing, defaults to 0
    """
    def __init__(self, nodetypes, initialLocation, **keyword_list):
        default_electromagnet_force = 1
        self.nodes = nodetypes #sets the node types, starting with the top one and
                               #going around counterclock wise
        self.loc = initialLocation #sets the loc of this instance of a weasel ball (length two list)
        self.velocity = 0
        if 'name' in keyword_list:
            self.name = keyword_list['name']
        if 'electromagnet_strength' in keyword_list:
            self.electromagnet_strength = keyword_list['electromagnet_strength']
        else:
            self.electromagnet_strength = default_electromagnet_force
        if 'orientation' in keyword_list:
            self.orientation = keyword_list['orientation']
        else: self.orientation = 0
        if 'velocity' in keyword_list:
            self.velocity = keyword_list['velocity']
    def _set_rotation(self):
        """This function will generation weighted rotation value
        that can be used to generate the new direction the
        ball is facing
        Random value generated is generated from a standardized
        normal gaussian distrubition"""
        random_val = np.random.standard_normal()
        rotation = random_val * 180
        self.orientation += rotation
    def contact_wall(self, wall, **keyword_list):
        """Protocol for figuring out what happens on wall impact
        currently just kills speed and figures out a new heading"""
        self.velocity = 0
        if 'deflection_protocol' in keyword_list:
            protocol = keyword_list['deflection_protocol']
        else:
            protocol = 0
        if protocol == 0:
            
        # Needs more work!

    def take_turn(self, iteration_length):
        self._set_rotation() #rotates ball

    def dump_stats(self):
        """This function is basically just for informational purposes,
        creates a dictionar full of all of this instances stats;
        could be used for gettings data, although not its purpose"""
        instance_stats = {
            'nodetypes' : self.nodes,
            'electromagnet_strength' : self.electromagnet_strength,
            'location' : self.loc,
            'velocity' : self.velocity,
            'orientaton' : self.orientation,
        }
        return instance_stats
    

class PlayGround(object): ##Creates a playspace for the balls to be in
    """Creates the environment for weasalball objects to
    play around in"""

    players = []
    def __init__(self, xbound, ybound, **keyword_list):
        self.xbound = xbound
        self.ybound = ybound
        for item in keyword_list:
            if item == 'players':
                self.players = keyword_list[item]
    def player_in_bound(self, ball): ##checks if a particular is in the playground, True
        "Tests in a ball is still within the valid playground"
        return bool(ball.loc[0] < self.xbound and ball.loc[1] < self.ybound)
    def add_player(self, new_player):
        "Adds a new player to the list of tracked players in the playground"
        self.players.append(new_player)
        


class module(object):
    """Defining the module class of objects
    modules can be attachted to a node of a WeaselBall
    and have specific characteristics such as bound range, strength, etc.
    Specific modules will exist, but they can be user creatable on the fly"""
    def __init__(self, module_identifier = "")