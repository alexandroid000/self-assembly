class Node(object):
    '''Node object that can connect to nodes of a weasel ball that affects behavoiur
    interactions. For now, default to electromagnet'''

    def __init__(self, type):
        self.type = type
        self.overWritesContactMethod = False
        self.isMagnetic = True