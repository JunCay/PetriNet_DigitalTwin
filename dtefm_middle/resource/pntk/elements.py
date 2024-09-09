import uuid

class Layout():
    def __init__(self, x=0, y=0, width=50, height=50, stroke_thick=1):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.stroke_thick = stroke_thick
        

class Place():
    def __init__(self, name, initial_marking, port_type=''):
        self.id = uuid.uuid4()
        self.name = name
        self.ins = dict()
        self.outs = dict()
        self.in_arcs = dict()
        self.out_arcs = dict()
        self.marking = initial_marking
        self.initial_marking = dict()
        for key in initial_marking.keys():
            self.initial_marking[key] = initial_marking[key]
        
    @property
    def tokens(self):
        return sum(self.marking.values())

    def initialize(self):
        for key in self.initial_marking.keys():
            self.marking[key] = self.initial_marking[key]
            
    def set_mark(self, new_mark):
        self.marking = dict()
        for key in new_mark.keys():
            self.marking[key] = new_mark[key]
    
class Transition():
    def __init__(self, name, condition=None, time=0.0, priority=0):
        self.id = uuid.uuid4()
        self.name = name
        self.ins = dict()
        self.outs = dict()
        self.in_arcs = dict()
        self.out_arcs = dict()
        self.condition = condition
        self.status = 'unready'
        self.work_status = 'unfiring'
        self.consumption = time
        self.time = 0.0           # rest_time
        self.priority = priority
        
    # The action of taking a marking from a source place occupies a marking resource
    def tick(self, dt):
        if self.work_status == 'firing':
            self.time -= dt
            if self.time <= 0.0:
                self.time = 0.0
                return True
            else:
                return False
        else:
            return False
        
    
class Arc():
    def __init__(self, node1, node2, annotation={'0':1}):
        self.id = uuid.uuid4()
        if isinstance(node1, Place) and isinstance(node2, Transition):
            self.direction = 'PtoT'
        elif isinstance(node2, Place) and isinstance(node1, Transition):
            self.direction = 'TtoP'
        else:
            print(f"Arc initialization failed")
            return
        self.node_in = node1
        self.node_out = node2
        self.name = f'{node1.name}->{node2.name}'
        self.annotation = annotation