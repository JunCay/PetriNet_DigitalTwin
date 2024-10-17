from elements import *
import numpy as np
import uuid
import csv
import ast

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
        self.dt = 0.01
        self.train_time = 0.0
        self.current_gesture = '0000'
        self.marking_types = list()
        self.reward_dict = {'progress': 10, 'fire': 2, 'unready_fire': -10, 'on_fire_fire': -20, 'duplicate_fire': -20, 'idle': -self.dt, 'fire_time_penalty': -self.dt*0.1}
        self.last_fire = None
        
    def __str__(self):
        res = []
        for p in self.places.values():
            res.append({p.name: p.marking})
        return str(res)
        
    def set_dt(self, dt):
        self.dt = dt
        self.reward_dict['idle'] = -self.dt
        self.reward_dict['fire_time_penalty'] = -self.dt*0.1
        
    def add_node(self, node):
        """
        Add node to the petri net.
        Then update the ready transition list.
        Args:
            node (Transition): The transition to be added.
            node (Place): The place to be added.
        Returns:
            None
        """
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
        """
        Add an arc to the petri net. Link TtoP or PtoT.
        Then update the ready transition list.

        Args:
            node1 (Transition / Place)
            node2 (Place / Transition)
        Returns:
            None 
        """
        arc = Arc(node1, node2)
        self.arcs[arc.name] = arc
        node1.outs[node2.id] = node2
        node1.out_arcs[arc.id] = arc
        node2.ins[node1.id] = node1
        node2.in_arcs[arc.id] = arc
        self.name_node[arc.name] = arc
        self.id_name[arc.id] = arc.name
        self.adj[(node1.id, node2.id)] = arc
        self.update_ready_transition()
    
    def init_by_csv(self, path):
        """
        Init the petri net by a csv file.

        Args:
            path (str): The path of the csv file.
        """
        with open(path, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                row = {k.strip(): v.strip() for k, v in row.items()}
                command = row['command']
                args_list = ast.literal_eval(row['args'])
                args = []
                for arg in args_list:
                    try:
                        a = ast.literal_eval(arg)
                        args.append(a)
                    except:
                        args.append(arg)
                if command == 'MADP':
                    # Add Place Node
                    # self.add_node(Place(args[0], ast.literal_eval(args[1]))) 
                    self.add_node(Place(*args))
                elif command == 'MADT':
                    # Add Transition Node
                    # self.add_node(Transition(args[0], None, ast.literal_eval(args[1])))
                    self.add_node(Transition(*args))  
                elif command == 'MADA':
                    # Add Arc
                    self.add_arc(self.name_node[args[1]], self.name_node[args[2]])
                elif command == 'MAST':
                    # Set target marking
                    self.name_node[args[0]].set_target_marking(args[1])
                    
        self.set_net_ready()
        
    @property
    def tokens(self):
        """
        Get the tokens of all places in the petri net.

        Returns:
            List[Dict]: The tokens of all places in the petri net.
        """
        res = []
        for p in self.places.values():
            res.append({'id':p.id, 'tokens':p.tokens, 'marking':p.marking})
        return res
        
    @property 
    def ready_transition(self):
        """
        Return the ready transition in the petri net.

        Returns:
            List[Transition]: by transition nodes.
        """
        res = []
        for t in self.transitions.values():
            if self.transition_ready_check(t):
                res.append(t)
        return res
        
    def update_ready_transition(self):
        """
        Update the ready transition in the petri net to transition nodes.
        """
        for t in self.transitions.values():
            self.transition_ready_check(t)


    def transition_ready_check(self, transition:Transition):
        """
        Check if the input transition is ready to fire.

        Args:
            transition (Transition): The transition to be checked.

        Returns:
            Boolean: if the transition is ready to fire.
        """
        for arc in transition.in_arcs.values():
            if not self.arc_ready(arc):
                # transition.status = 'unready'
                transition.set_status('unready')
                return False
            
        for arc in transition.out_arcs.values():
            if not self.arc_ready(arc):
                # transition.status = 'unready'
                transition.set_status('unready')
                return False
        # transition.status = 'ready'
        transition.set_status('ready')
        return True
    
    def arc_ready(self, arc:Arc):
        """
        Check if the input arc is ready to fire. 
        Arc stresses the requirement of the transition.

        Args:
            arc (Arc): The arc to be checked.

        Returns:
            Boolean: if the arc is ready to fire.
        """
        if arc.direction == 'PtoT':
            if arc.node_in.time > 0:
                return False
            for k in arc.annotation.keys():
                if arc.node_in.marking[k] - arc.annotation[k] >= 0:
                    return True
            return False
        elif arc.direction == 'TtoP':
            if arc.node_out.place_type == 'resource':
                if arc.node_out.tokens > 0:
                    return False
                return True
            else:
                return True
    
    def fire_transition(self, transition:Transition):
        """
        Fire the input transition. Then update the ready transition list.
        Used when the net is not a timed net.
        
        Args:
            transition (Transition): The transition to be fired.

        Returns:
            Boolean: if the transition is successfully fired.
        """
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
        """
        Set the transition to firing status.

        Args:
            transition (Transition): The transition to be started.

        Returns:
            Boolean: if the transition is successfully started.
        """
        # print(f"on firing transition: {transition.name}")
        if transition not in self.transitions.values():
            if self.debug:
                print(f"Transition not found in current petri net")
            return False
        if not self.transition_ready_check(transition):
            if self.debug:
                print(f"Transition not ready")
            return False
        if transition.work_status == 'firing':
            if self.debug:
                print(f"Transition is already firing")
            return False
        
        for arc in self.transitions[transition.id].in_arcs.values():
            for k in arc.node_in.marking.keys():
                arc.node_in.marking[k] -= arc.annotation[k]
        # transition.work_status = 'firing'
        # transition.time = transition.consumption
        if transition.consumption >= 0.001:
            transition.set_on_fire(self.current_gesture)
        else:
            for arc in self.transitions[transition.id].out_arcs.values():
                print(f"{arc.node_out.name} : {arc.node_out.visibility}")
                if arc.node_out.visibility == 'visible':
                    continue
                for k in arc.node_out.marking.keys():
                    arc.node_out.marking[k] += arc.annotation[k]
            self.update_gesture(transition.target_gesture)
            transition.work_status = 'unfiring'
            self.update_ready_transition()
            
        return True
    
    def on_fire_transition_restrict(self, transition:Transition):
        """
        Set the transition to firing status. Except influnce visible.

        Args:
            transition (Transition): The transition to be started.

        Returns:
            Boolean: if the transition is successfully started.
        """
        # print(f"on firing transition: {transition.name}")
        if transition not in self.transitions.values():
            if self.debug:
                print(f"Transition not found in current petri net")
            return False
        if not self.transition_ready_check(transition):
            if self.debug:
                print(f"Transition not ready")
            return False
        if transition.work_status == 'firing':
            if self.debug:
                print(f"Transition is already firing")
            return False
        
        for arc in self.transitions[transition.id].in_arcs.values():
            if arc.node_in.visibility == 'visible':
                continue
            for k in arc.node_in.marking.keys():
                arc.node_in.marking[k] -= arc.annotation[k]
        transition.set_on_fire()
        return True
    
    def off_fire_transition_restrict(self, transition:Transition, debug):
        """
        Set the transition to firing status. Except influnce visible.

        Args:
            transition (Transition): The transition to be started.

        Returns:
            Boolean: if the transition is successfully started.
        """
        # print(f"set off firing transition: {transition.name}")
        if transition not in self.transitions.values():
            if debug:
                print(f"Transition not found in current petri net")
            return False

        # print("out arcs: ", self.transitions[transition.id].out_arcs)
        for arc in self.transitions[transition.id].out_arcs.values():
            print(f"{arc.node_out.name} : {arc.node_out.visibility}")
            if arc.node_out.visibility == 'visible':
                continue
            for k in arc.node_out.marking.keys():
                arc.node_out.marking[k] += arc.annotation[k]
        transition.work_status = 'unfiring'
        self.update_ready_transition()
        return True
    
    def chech_alive(self):
        self.update_ready_transition()
        for t in self.transitions.values():
            if t.status == 'ready':
                return True
            else:
                if t.work_status == 'firing':
                    return True
        return False
    
    def tick(self, dt):
        """
        Tick the time of all transitions.

        Args:
            dt (float): delta time

        Returns:
            List[str]: The names of the finished transitions
        """
        self.train_time += dt
        changed = []
        for transition in self.transitions.values():
            f = transition.tick(dt)
            if f:
                changed.append(transition.name)
                for arc in transition.out_arcs.values():
                    for k in arc.node_out.marking.keys():
                        arc.node_out.marking[k] += arc.annotation[k]
                transition.work_status = 'unfiring'
                # print(transition.name, transition.target_gesture)
                self.update_gesture(transition.target_gesture)
                self.update_ready_transition()
        return changed
    
    def update_gesture(self, gesture):
        new_gesture = []
        gesture = str(gesture)
        if len(gesture) != 4:
            raise Exception("The gesture should be 4 characters.")
        for i in range(len(gesture)):
            if gesture[i] == '-':
                new_gesture.append(self.current_gesture[i])
            else:
                new_gesture.append(gesture[i])
        self.current_gesture = ''.join(new_gesture)
            
    def reset_net(self):
        """
        Reset the marking of all places in the net to initial marking.
        """
        for place in self.places.values():
            place.initialize()
        
    def reset_transitions(self):
        for transition in self.transitions.values():
            transition.status = 'unready'
            transition.work_status = 'unfiring'
            transition.time = 0.0
    
    def get_marking_types(self):
        for p in self.places.values():
            for t in p.marking.keys():
                if t not in self.marking_types:
                    self.marking_types.append(t)
        return self.marking_types
    
    def print_adj(self):
        res = []
        res.append(['\\'])
        for i, key_from in enumerate(self.name_node.keys()):
            if isinstance(self.name_node[key_from], Place) or isinstance(self.name_node[key_from], Transition):
                res[0].append(key_from)
                res.append([key_from])
                for j, key_to in enumerate(self.name_node.keys()):
                    if isinstance(self.name_node[key_to], Place) or isinstance(self.name_node[key_to], Transition):
                        if ((self.name_node[key_from].id, self.name_node[key_to].id) in self.adj.keys()):
                            res[i+1].append(1)
                        else:
                            res[i+1].append(0)
        mat_str = 'identity petri net adjMatrix: \n'
        for i in range(len(res)):
            for j in range(len(res[0])):
                mat_str += str(res[i][j]) + '\t'
            mat_str += '\n'
        print(mat_str)
        
    def set_net_ready(self):
        self.adj_matrix = np.zeros((len(self.places), len(self.transitions)+1))
        for i, place in enumerate(self.places.values()):
            for j, transition in enumerate(self.transitions.values()):
                if place.name + '->' + transition.name in self.arcs.keys():
                    self.adj_matrix[i][j] = -1
                if transition.name + '->' + place.name in self.arcs.keys():
                    self.adj_matrix[i][j] = 1
                    
        self.get_action_space()
        self.update_ready_transition()
    
    def get_adj_matrix(self):
        self.set_net_ready()
        return self.adj_matrix
    
    # RL part
    def reset(self):
        """
        Reset the net to initial state
        """
        self.train_time = 0.0
        self.reset_net()
        self.reset_transitions()
        self.update_ready_transition()
        
        return self.get_state()

    def set_reward(self, type, reward):
        self.reward_dict[type] = reward
    
    def get_action_space(self):
        self.action_list = []
        for transition in self.transitions.values():
            # if transition.consumption > 0.0:
            #     self.action_list.append(transition)
            self.action_list.append(transition)
        return len(self.transitions)+1
    
    def get_place_colored_state(self):
        """
        Tobe overrided to customize the state of the petri net.

        Returns:
            np.ndarray: state matrix of places
        """
        marking_types = self.get_marking_types()
        place_state = np.zeros((len(self.places), len(marking_types)))
        target_state = np.zeros((len(self.places), len(marking_types)))
        place_time_state = np.zeros((len(self.places), len(marking_types)))
        for i, place in enumerate(self.places.values()):
            for mt in marking_types:
                if mt in place.marking.keys():
                    place_state[i, marking_types.index(mt)] = place.marking[mt]
                    target_state[i, marking_types.index(mt)] = place.target_marking[mt]
        # res_state = np.stack((place_state, target_state), axis=1)
        res_state = np.concatenate((place_state, target_state), axis=1)
        res_state = np.concatenate((res_state, place_time_state), axis=1)
        return res_state
    
    def get_place_neural_state(self):
        marking_types = self.get_marking_types()
        place_state = np.zeros((len(self.places), 3))
        for i, place in enumerate(self.places.values()):
            place_state[i, 0] = place.tokens
            place_state[i, 1] = place.target_tokens
            if place.place_type == 'resource':
                place_state[i, 2] = 1.0
        return place_state
    
    def get_transition_neural_state(self):
        """
        Tobe overrided to customize the state of the petri net.

        Returns:
            _type_: _description_
        """
        trans_state = np.zeros((len(self.transitions)+1, 3))
        for i, transition in enumerate(self.transitions.values()):
            if transition.status == 'ready':
                trans_state[i, 0] = 1.0
            else:
                trans_state[i, 0] = 0.0
            if transition.work_status == 'firing':
                trans_state[i, 1] = 1.0
                if transition.this_consumption >= 0.001:
                    trans_state[i, 2] = (transition.this_consumption - transition.time) / transition.this_consumption
                else:
                    trans_state[i, 2] = 0
        trans_state[-1] = [1.0, 0.0, 0.0]       # add idle transition
        return trans_state
    
    def get_state(self):
        place_state = self.get_place_neural_state()
        trans_state = self.get_transition_neural_state()
        # state = np.concatenate((place_state, trans_state), axis=0)
        return [place_state, trans_state]
        # return state
    
    def get_state_space(self):
        return self.get_state()[0].shape, self.get_state()[1].shape
    
    def step(self, action, debug=False):
        reward_dict = {}
        p_state = self.get_state()[0]
        t_state = self.get_state()[1]
        p_dist = np.sum(np.abs(p_state[:, 1] - p_state[:, 0]) * p_state[:, 2])
        fire_ts = np.sum(t_state[:, 1])
        reward_dict['fire_time_penalty'] = fire_ts * self.reward_dict['fire_time_penalty']
        
        if isinstance(action, int):
            if action == len(self.transitions) or action == -1:
                trans = None
            else:
                trans = self.action_list[action]
            
            if trans == None:
                reward_dict['idle'] = self.reward_dict['idle']
                
            else:
                if not self.transition_ready_check(trans):
                    if trans == self.last_fire:
                        reward_dict['duplicate_fire'] = self.reward_dict['duplicate_fire']
                    reward_dict['unready_fire'] = self.reward_dict['unready_fire']
                    
                else:
                    if self.on_fire_transition(trans):
                        reward_dict['fire'] = self.reward_dict['fire']
                        reward_dict['bonus'] = trans.bonus
                    else:
                        reward_dict['on_fire_fire'] = self.reward_dict['on_fire_fire']
                self.last_fire = trans

            
            self.update_ready_transition()
            next_state = self.get_state()
            p_state_ = next_state[0]
            p_dist_ = np.sum(np.abs(p_state_[:, 1] - p_state_[:, 0]) * p_state_[:, 2])
            reward_dict['progress'] = (p_dist - p_dist_) * self.reward_dict['progress']
            reward = sum(reward_dict.values())
            done = not self.chech_alive()
            
            self.tick(self.dt)
            
            # print(reward_dict)
            
            
        if debug:
            return next_state, reward, done, reward_dict
        else:
            return next_state, reward, done
        
        
        