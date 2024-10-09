import math
import sys
import os
import numpy as np
import torch
import torch.nn as nn
from torch.nn import functional as F
from torch.distributions import Categorical
import matplotlib.pyplot as plt
import collections
import random

from tqdm import tqdm
import rl_utils
from gcpn import *

from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('dtefm_middle')
library_path = os.path.join(package_share_directory, 'resource/pntk')
sys.path.append(library_path)
from elements import *
from petri_net import *
from example_net import *

class GPNPolicyNet(torch.nn.Module):
    def __init__(self, lp0, lt0, adj_matrix, device):
        super(GPNPolicyNet, self).__init__()
        self.adj_matrix = torch.from_numpy(adj_matrix).float().to(device)
        
        self.gcpn_1 = GCPN_layer(lp0, lt0, 8, 8, self.adj_matrix)
        self.gcpn_2 = GCPN_layer(8, 8, 16, 16, self.adj_matrix)
        self.gcpn_3 = GCPN_layer(16, 16, 64, 64, self.adj_matrix)
        self.gcpn_4 = GCPN_layer(64, 64, 16, 16, self.adj_matrix)
        self.ac = AC_layer(16, 16, self.adj_matrix)
        

    def forward(self, p, t):
        if isinstance(p, np.ndarray):
            p = torch.from_numpy(p).float()
        if isinstance(t, np.ndarray):
            t = torch.from_numpy(t).float()
        p1, t1 = self.gcpn_1(p, t)
        p2, t2 = self.gcpn_2(p1, t1)
        p3, t3 = self.gcpn_3(p2, t2)
        p4, t4 = self.gcpn_4(p3, t3)
        tf = self.ac(p4, t4)
        # print(tf)
        
        return tf

class GPNCriticNet(torch.nn.Module):
    def __init__(self, lp0, lt0, adj_matrix, device):
        super(GPNCriticNet, self).__init__()
        self.adj_matrix = torch.from_numpy(adj_matrix).float().to(device)
        
        self.gcpn_1 = GCPN_layer(lp0, lt0, 8, 8, self.adj_matrix)
        self.gcpn_2 = GCPN_layer(8, 8, 16, 16, self.adj_matrix)
        self.gcpn_3 = GCPN_layer(16, 16, 64, 64, self.adj_matrix)
        self.sv = SV_layer(64, 64, self.adj_matrix)
        

    def forward(self, p, t):
        if isinstance(p, np.ndarray):
            p = torch.from_numpy(p).float()
        if isinstance(t, np.ndarray):
            t = torch.from_numpy(t).float()
        p1, t1 = self.gcpn_1(p, t)
        p2, t2 = self.gcpn_2(p1, t1)
        p3, t3 = self.gcpn_3(p2, t2)
        out = self.sv(p3, t3)
        # print(out)
        return out

class Agent_PPO():
    def __init__(self, lp, lt, action_dim, adj_matrix, actor_lr, critic_lr, lmbda, epochs, epsilon, eps, gamma, device):
        self.action_dim = action_dim
        self.adj_matrix = torch.from_numpy(adj_matrix).float().to(device)
        self.actor = GPNPolicyNet(lp, lt, adj_matrix, device).to(device)
        self.critic = GPNCriticNet(lp, lt, adj_matrix, device).to(device)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr)
        self.gamma = gamma
        self.lmbda = lmbda
        self.epochs = epochs
        self.epsilon = epsilon
        self.eps = eps      # PPO-Clip epsilon
        self.device = device

    def take_action(self, state):
        if np.random.random() < self.epsilon:
            if self.epsilon > 0.00001:
                self.epsilon *= 0.99
            action = np.random.randint(0, self.action_dim)
            return action
        else:
            state_p = state[0]
            state_t = state[1]
            choix = np.where((state_t[:, 0] == 1) & (state_t[:, 1] == 0))[0]
            print(f"current choix: {choix}")
            choix = torch.tensor(choix).to(self.device)
            
            state_p = torch.tensor([state_p], dtype=torch.float).to(self.device)
            state_t = torch.tensor([state_t], dtype=torch.float).to(self.device)
            
            probs = self.actor(state_p, state_t)
            # noise = torch.ones_like(probs) - torch.randn_like(probs) * 0.1
            # probs *= noise
            mask = torch.zeros_like(probs)
            mask[-1, choix] = 1
            probs = probs * mask
            # print(probs)
            # print(choix)
            # print(mask)
            
            sums= probs.sum(dim=1, keepdim=True)
            probs = torch.div(probs, sums)
            
            try:
                action_list = torch.distributions.Categorical(probs)
            except:
                # print(self.actor.state_dict())
                # print(self.critic.state_dict())
                # print("probs: ", probs)
                # print("choix: ", choix)
                print(state_p)
                pass
            
            action = action_list.sample()
            # print(action)
            return action.item()

    def update(self, transition_dict):
        # states = torch.tensor(transition_dict['states'], dtype=torch.float).to(self.device)
        p_states = [item[0] for item in transition_dict['states']]
        t_states = [item[1] for item in transition_dict['states']]
        
        p_states = torch.tensor(p_states, dtype=torch.float).squeeze(0).to(self.device)
        t_states = torch.tensor(t_states, dtype=torch.float).squeeze(0).to(self.device)
        
        actions = torch.tensor(transition_dict['actions']).view(-1, 1).to(self.device)
        rewards = torch.tensor(transition_dict['rewards'], dtype=torch.float).view(-1, 1).to(self.device)
        # next_states = torch.tensor(transition_dict['next_states'], dtype=torch.float).to(self.device)
        next_p_states = [item[0] for item in transition_dict['next_states']]
        next_t_states = [item[1] for item in transition_dict['next_states']]
        
        next_p_states = torch.tensor(p_states, dtype=torch.float).squeeze(0).to(self.device)
        next_t_states = torch.tensor(t_states, dtype=torch.float).squeeze(0).to(self.device)
        
        dones = torch.tensor(transition_dict['dones'], dtype=torch.float).view(-1, 1).to(self.device)
        td_target = rewards + self.gamma * self.critic(next_p_states, next_t_states) * (1-dones)
        td_delta = td_target - self.critic(p_states, t_states)
        advantage = rl_utils.compute_advantage(self.gamma, self.lmbda, td_delta.cpu()).to(self.device)
        old_log_probs = torch.log(self.actor(p_states, t_states).gather(1, actions) + 1e-10).detach()
        # print(self.critic(p_states, t_states))
        
        for _ in range(self.epochs):
            log_probs = torch.log(self.actor(p_states, t_states).gather(1, actions) + 1e-10)
            
            ratio = torch.exp(log_probs - old_log_probs)
            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1-self.eps, 1+self.eps)
            
            actor_loss = torch.mean(-torch.min(surr1, surr2))
            # print(self.actor(p_states, t_states), actor_loss)
            critic_loss = torch.mean(F.mse_loss(self.critic(p_states, t_states), td_target.detach()))
            # print(critic_loss)
            self.actor_optimizer.zero_grad()
            self.critic_optimizer.zero_grad()
            actor_loss.backward()
            # torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0) 
            critic_loss.backward()
            
            self.actor_optimizer.step()
            self.critic_optimizer.step()
            # for name, param in self.actor.named_parameters():
            #     if param.grad is not None:
            #         print(f'Parameter: {name}, Gradient norm: {torch.norm(param.grad)}')
