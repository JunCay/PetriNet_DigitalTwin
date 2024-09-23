from elements import *
from petri_net import *

class PlainNet(ColoredPetriNet):
    def __init__(self, name):
        super().__init__(name)

class TestNet(ColoredPetriNet):
    def __init__(self, name):
        super().__init__(name)
        self.p1 = Place('p1', {'0': 25})
        self.p2 = Place('p2', {'0': 0})
        self.p3 = Place('p3', {'0': 0})
        self.t1 = Transition('t1')
        self.t2 = Transition('t2')
        self.add_node(self.t1)
        self.add_node(self.t2)
        self.add_node(self.p1)
        self.add_node(self.p2)
        self.add_node(self.p3)
        self.add_arc(self.p1, self.t1)
        self.add_arc(self.t1, self.p2)
        self.add_arc(self.p2, self.t2)
        self.add_arc(self.t2, self.p3)