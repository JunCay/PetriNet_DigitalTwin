import math
import numpy as np
import torch
import torch.nn as nn
from torch.nn import functional as F
from torch.distributions import Categorical

# Single GCPN Layer
class P2P(nn.Module):
    def __init__(self, dim_in, dim_out, adj_pt, f=F.leaky_relu):
        super(P2P, self).__init__()
        self.weight = nn.Parameter(torch.Tensor(dim_out, dim_in).float())
        self.bias = nn.Parameter(torch.Tensor(dim_out).float())
        self.f = f
        self.adj_pt = adj_pt
        self.pm = torch.matmul(self.adj_pt, self.adj_pt.t())
        self.batch_norm = nn.BatchNorm1d(dim_out)
        self.reset_parameters()
    
    def reset_parameters(self):
        nn.init.kaiming_uniform_(self.weight, a=math.sqrt(5))
        if self.bias is not None:
            fan_in, _ = nn.init._calculate_fan_in_and_fan_out(self.weight)
            bound = 1 / math.sqrt(fan_in)
            nn.init.uniform_(self.bias, -bound, bound)
    
    def forward(self, x):
        x = torch.matmul(self.pm, x)
        diag = self.pm.diagonal().unsqueeze(0).unsqueeze(2)
        x = x / diag
        x = torch.matmul(x, self.weight.t()) + self.bias.unsqueeze(0)
        
        batch_size, Np, dim_out = x.size()
        x = x.view(-1, dim_out)
        x = self.batch_norm(x)
        x = x.view(batch_size, Np, dim_out) 
        
        x = self.f(x)
        return x
    
class T2T(nn.Module):
    def __init__(self, dim_in, dim_out, adj_pt, f=F.leaky_relu):
        super(T2T, self).__init__()
        self.weight = nn.Parameter(torch.Tensor(dim_out, dim_in).float())
        self.bias = nn.Parameter(torch.Tensor(dim_out).float())
        self.f = f
        self.adj_pt = adj_pt
        self.pm = torch.matmul(self.adj_pt.t(), self.adj_pt)
        self.batch_norm = nn.BatchNorm1d(dim_out)
        self.reset_parameters()
    
    def reset_parameters(self):
        nn.init.kaiming_uniform_(self.weight, a=math.sqrt(5))
        if self.bias is not None:
            fan_in, _ = nn.init._calculate_fan_in_and_fan_out(self.weight)
            bound = 1 / math.sqrt(fan_in)
            nn.init.uniform_(self.bias, -bound, bound)
    
    def forward(self, x):
        x = torch.matmul(self.pm, x)
        
        diag = self.pm.diagonal()+1
        diag = diag.unsqueeze(0).unsqueeze(2)
        x = x / diag
        x = torch.matmul(x, self.weight.t()) + self.bias.unsqueeze(0)
        
        batch_size, Np, dim_out = x.size()
        x = x.view(-1, dim_out)
        x = self.batch_norm(x)
        x = x.view(batch_size, Np, dim_out) 
        
        x = self.f(x)
        return x
    
class T2P(nn.Module):
    def __init__(self, dim_in, dim_out, adj_pt, f=F.leaky_relu, add_bias=True):
        super(T2P, self).__init__()
        self.adj_pt = adj_pt
        self.f = f
        self.weight = nn.Parameter(torch.Tensor(dim_out, dim_in).float())
        self.bias = nn.Parameter(torch.Tensor(dim_out).float())
        self.add_bias = add_bias
        self.batch_norm = nn.BatchNorm1d(dim_out)
        
        self.reset_parameters()
    
    def reset_parameters(self):
        nn.init.kaiming_uniform_(self.weight, a=math.sqrt(5))
        if self.bias is not None:
            fan_in, _ = nn.init._calculate_fan_in_and_fan_out(self.weight)
            bound = 1 / math.sqrt(fan_in)
            nn.init.uniform_(self.bias, -bound, bound)
            
    def forward(self, x):
        x = torch.matmul(self.adj_pt, x)
        x = torch.matmul(x, self.weight.t())
        if self.add_bias:
            x = x + self.bias.unsqueeze(0)
            
        batch_size, Np, dim_out = x.size()
        x = x.view(-1, dim_out)
        x = self.batch_norm(x)
        x = x.view(batch_size, Np, dim_out) 
            
        x = self.f(x)
        return x
        
class P2T(nn.Module):
    def __init__(self, dim_in, dim_out, adj_pt, f=F.leaky_relu, add_bias=True):
        super(P2T, self).__init__()
        self.adj_pt = adj_pt
        self.f = f
        self.weight = nn.Parameter(torch.Tensor(dim_out, dim_in).float())
        self.bias = nn.Parameter(torch.Tensor(dim_out).float())
        self.add_bias = add_bias
        self.batch_norm = nn.BatchNorm1d(dim_out)
        self.reset_parameters()
        
    def reset_parameters(self):
        nn.init.kaiming_uniform_(self.weight, a=math.sqrt(5))
        if self.bias is not None:
            fan_in, _ = nn.init._calculate_fan_in_and_fan_out(self.weight)
            bound = 1 / math.sqrt(fan_in)
            nn.init.uniform_(self.bias, -bound, bound)
        
    def forward(self, x):
        x = torch.matmul(self.adj_pt.t(), x)
        x = torch.matmul(x, self.weight.t())
        if self.add_bias:
            x = x + self.bias.unsqueeze(0)
            
        batch_size, Np, dim_out = x.size()
        x = x.view(-1, dim_out)
        x = self.batch_norm(x)
        x = x.view(batch_size, Np, dim_out) 
        
        x = self.f(x)
        return x
        
# GCPN Group Layers 
class GCPN_layer(torch.nn.Module):
    def __init__(self, lp_in, lt_in, lp_out, lt_out, adj_matrix):
        super(GCPN_layer, self).__init__()
        self.P2P = P2P(lp_in, lp_out, adj_matrix)
        self.T2T = T2T(lt_in, lt_out, adj_matrix)
        self.P2T = P2T(lp_out, lt_out, adj_matrix)
        self.T2P = T2P(lt_out, lp_out, adj_matrix)
        
    def forward(self, p, t):
        p1 = self.P2P(p)
        t1 = self.T2T(t)
        
        dt = self.P2T(p1)
        t2 = t1 + dt
        
        dp = self.T2P(t2)
        p2 = p1 + dp
        
        return p2, t2
    
# Output Layers
class SV_layer(torch.nn.Module):
    def __init__(self, lp_in, lt_in, adj_matrix):
        super(SV_layer, self).__init__()
        self.p_dim = adj_matrix.shape[0]
        self.T2P = T2P(lt_in, lp_in, adj_matrix)
        self.fc = nn.Linear(self.p_dim * lp_in, 1)
        
    def forward(self, p, t):
        dp = self.T2P(t)
        p1 = p+dp
        
        batch_size = p1.size(0)
        
        p1 = p1.view(batch_size, -1)
        out = self.fc(p1)
        return out
        
class AC_layer(torch.nn.Module):
    def __init__(self, lp_in, lt_in, adj_matrix):
        super(AC_layer, self).__init__()
        self.t_dim = adj_matrix.shape[1]
        self.lt_in = lt_in
        self.P2T = P2T(lp_in, lt_in, adj_matrix)
        self.fc = nn.Linear(self.t_dim * lt_in, self.t_dim)
        self.softmax = nn.Softmax(dim=-1)
        
    def forward(self, p, t):
        dt = self.P2T(p)
        t1 = t+dt
        
        batch_size = t1.size(0)
        
        t1 = t1.view(batch_size, self.t_dim * self.lt_in)
        out = self.fc(t1)
        out = F.softmax(out, dim=-1)
        # out = self.softmax(t1)
        # print(p, out)
        return out