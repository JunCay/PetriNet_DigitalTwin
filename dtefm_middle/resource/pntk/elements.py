import uuid

class Layout():
    def __init__(self, x=0, y=0, width=50, height=50, stroke_thick=1):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.stroke_thick = stroke_thick
        

class Place():
    def __init__(self, name, initial_marking, processing_time=0.0, visibility='unvisible', place_type='activity'):
        self.id = uuid.uuid4()
        self.name = name
        self.ins = dict()
        self.outs = dict()
        self.in_arcs = dict()
        self.out_arcs = dict()
        self.marking = initial_marking.copy()
        self.initial_marking = initial_marking.copy()
        self.target_marking = dict()
        self.processing_time = processing_time
        self.time = 0.0
        self.visibility = visibility
        self.place_type = place_type
        self.set_initial_marking(initial_marking)
        self.set_target_marking(initial_marking)

    @property
    def tokens(self):
        """
        Returns:
            int: The sum of number of tokens in the place
        """
        return sum(self.marking.values())
    
    @property
    def target_tokens(self):
        """
        Returns:
            int: The sum of number of tokens in the place
        """
        return sum(self.target_marking.values())

    def initialize(self):
        """
        Set the marking to initial marking
        """
        for key in self.initial_marking.keys():
            self.marking[key] = self.initial_marking[key]
            
    def set_mark(self, new_mark):
        """
        Set the marking to new_mark

        Args:
            new_mark (dict): kvp of mark kind: number
        """
        self.marking = dict()
        for key in new_mark.keys():
            self.marking[key] = new_mark[key]
    
    
    def set_initial_marking(self, new_mark):
        """
        Set the initial marking to new_mark
        Should not be called after the initial marking is set

        Args:
            new_mark (Dict): kvp of mark kind: number
        """
        for key in new_mark.keys():
            self.initial_marking[key] = new_mark[key]
            
    def set_target_marking(self, new_mark):
        """
        Set the target marking to new_mark

        Args:
            new_mark (dict): kvp of mark kind: number
        """
        for key in new_mark.keys():
            self.target_marking[key] = new_mark[key]
            
    def add_mark(self, new_mark):
        """
        Add new_mark to the marking

        Args:
            new_mark (dict): kvp of mark kind: number
        """
        for key in new_mark.keys():
            if key in self.marking.keys():
                self.marking[key] += new_mark[key]
            else:
                self.marking[key] = new_mark[key]
                
    def add_one_mark(self, mark_kind):
        if mark_kind in self.marking.keys():
            self.marking[mark_kind] += 1
        else:
            self.marking[mark_kind] = 1
    
    def set_on_process(self, processing_time):
        self.processing_time = processing_time

    def tick(self, dt):
        """
        Tick the time of the place.
        Args:
            dt (float): delta time
        """
        self.time -= dt
        if self.time <= 0.0:
            self.time = 0.0
            return True
        else:
            return False
    
class Transition():
    def __init__(self, name, time=0.0, target_gesture=None, bonus=0):
        self.id = uuid.uuid4()
        self.name = name
        self.ins = dict()
        self.outs = dict()
        self.in_arcs = dict()
        self.out_arcs = dict()
        self.status = 'unready'
        self.work_status = 'unfiring'
        self.init_time = 4.0
        self.target_gesture = target_gesture
        self.consumption = time
        self.this_consumption = self.consumption
        self.time = 0.0           # rest_time
        self.bonus = bonus
        
    # The action of taking a marking from a source place occupies a marking resource
    def tick(self, dt=0.01):
        """
        Tick the time of the transition.
        Reduce the rest time of the transition by dt.
        Args:
            dt (float): delta time

        Returns:
            bool: if the transition is finished.
        """
        if self.work_status == 'firing':
            self.time -= dt
            if self.time <= 0.0:
                self.time = 0.0
                return True
            else:
                return False
        else:
            return False
        
    def set_status(self, status):
        self.status = status
        
    def set_on_fire(self, current_gesture=None):
        self.work_status = 'firing'
        self.status = 'unready'
        self.time = self.consumption
        
        if current_gesture is not None:
            if current_gesture != self.target_gesture and self.target_gesture != '----':
                self.time += self.init_time
        self.this_consumption = self.time
        
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