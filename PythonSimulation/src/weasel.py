import numpy as np
import playGround as pg
import node as n

class WeaselBall(object):
    '''Creates the weaselball object, which is in charger of managing all attatched modules
    and holding information about where the ball is located and what is it doing
    as well as other important information'''

    def __init__(self, name, initialLocation, **keyword_list):
        dn = n.Node('default')
        self.node_types = [dn,dn,dn,dn]
        self.name = name
        self.xloc = initialLocation['xloc']
        self.yloc = initialLocation['yloc'] ### scrapes location from initialLocation Vector
        DEFAULT_SIZE = 1 ## change/fix and whatever
        self.orientation = 0.
        self.speed = 0
        self.size = DEFAULT_SIZE
        self.largest_valid_rotation = 30
        self.velocity = 0
        self.orientation = 0
        self.connectBalls = []
        if 'node_types' in keyword_list: ### scrape node types from input
            input_nodes = keyword_list['node_types']
            for index in range(0, len(keyword_list['node_types'])):
                self.node_types[index] = input_nodes[index]
        for item in keyword_list:
            if item == 'orientation':
                self.orientation = keyword_list['orientation']
            if item == 'velocity':
                self.velocity = keyword_list['velocity']
            if item == 'size':
                self.size = keyword_list['size']
            if item == 'max_speed':
                self.max_speed = keyword_list['max_speed']

    def dump_stats(self):
        '''This function can be used to scrape a bunch of
        stats from a WeaselBall
        Returns in a single dict with various variables and keywords'''
        nodes = []
        for node in self.node_types:
            nodes.append(node.type)
        instance_stats = {
            'nodes' : nodes,
            'xlocation' : self.xloc,
            'ylocation' : self.yloc,
            'velocity'  : self.velocity,
            'orientation' : self.orientation,
        }
        return instance_stats