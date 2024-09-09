from elements import *

class ColoredPetriNet():
    def __init__(self, name):
        self.id = uuid.uuid4()
        self.name = name
        self.places = dict()
        self.transitions = dict()
        self.arcs = dict()
        self.adj = dict()
        self.name_node = dict()
        self.id_name = dict()
        self.debug = False
        self.update_ready_transition()
        
    def __str__(self):
        res = []
        for p in self.places.values():
            res.append({p.name: p.marking})
        return str(res)
        
    def add_node(self, node):
        if isinstance(node, Place):
            self.places[node.id] = node
            self.name_node[node.name] = node
            self.id_name[node.id] = node.name
        elif isinstance(node, Transition):
            self.transitions[node.id] = node
            self.name_node[node.name] = node
            self.id_name[node.id] = node.name
        else:
            if self.debug:
                print("Node adding failed")
        self.update_ready_transition()
    
    def add_arc(self, node1, node2):
        arc = Arc(node1, node2)
        self.arcs[arc.id] = arc
        node1.outs[node2.id] = node2
        node1.out_arcs[arc.id] = arc
        node2.ins[node1.id] = node1
        node2.in_arcs[arc.id] = arc
        self.name_node[arc.name] = arc
        self.id_name[arc.id] = arc.name
        self.adj[(node1.id, node2.id)] = arc
        self.update_ready_transition()
        
    @property
    def tokens(self):
        res = []
        for p in self.places.values():
            res.append({'id':p.id, 'tokens':p.tokens, 'marking':p.marking})
        return res
        
    @property 
    def ready_transition(self):
        res = []
        for t in self.transitions.values():
            if self.transition_ready_check(t):
                res.append(t)
        return res
        
    def update_ready_transition(self):
        for t in self.transitions.values():
            self.transition_ready_check(t)


    def transition_ready_check(self, transition:Transition):
        for arc in transition.in_arcs.values():
            if not self.arc_ready(arc):
                transition.status = 'unready'
                return False
        transition.status = 'ready'
        return True
    
    def arc_ready(self, arc:Arc):
        if arc.direction == 'PtoT':
            for k in arc.annotation.keys():
                if arc.node_in.marking[k] - arc.annotation[k] >= 0:
                    return True
            return False
        else:
            return False
        
    def fire_transition(self, transition:Transition):
        # print(f"firing transition: {transition.name}")
        if transition not in self.transitions.values():
            if self.debug:
                print(f"Transition not found in current petri net")
            return False
        if not self.transition_ready_check(transition):
            if self.debug:
                print(f"Transition not found in current petri net")
            return False
        for arc in self.transitions[transition.id].in_arcs.values():
            for k in arc.node_in.marking.keys():
                arc.node_in.marking[k] -= arc.annotation[k]
        for arc in self.transitions[transition.id].out_arcs.values():
            for k in arc.node_out.marking.keys():
                arc.node_out.marking[k] += arc.annotation[k]

        self.update_ready_transition()
        return True

    def on_fire_transition(self, transition:Transition):
        # print(f"on firing transition: {transition.name}")
        if transition not in self.transitions.values():
            if self.debug:
                print(f"Transition not found in current petri net")
            return False
        if not self.transition_ready_check(transition):
            if self.debug:
                print(f"Transition not found in current petri net")
            return False
        if transition.work_status == 'firing':
            if self.debug:
                print(f"Transition is already firing")
            return False
        
        for arc in self.transitions[transition.id].in_arcs.values():
            for k in arc.node_in.marking.keys():
                arc.node_in.marking[k] -= arc.annotation[k]
        transition.work_status = 'firing'
        transition.time = transition.consumption
        return True
    
    def tick(self, dt):
        changed = []
        for transition in self.transitions.values():
            f = transition.tick(dt)
            if f:
                changed.append(transition.name)
                for arc in transition.out_arcs.values():
                    for k in arc.node_out.marking.keys():
                        arc.node_out.marking[k] += arc.annotation[k]
                transition.work_status = 'unfiring'
                self.update_ready_transition()
        return changed

    def reset_net(self):
        for place in self.places.values():
            place.initialize()
            