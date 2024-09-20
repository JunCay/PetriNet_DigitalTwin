import rclpy
import sys
import os
import ast

from rclpy.node import Node
from dtefm_interfaces.srv import PNCommand

import numpy as np
import torch
from torch.nn import functional as F
from torch.distributions import Categorical
import matplotlib.pyplot as plt

from tqdm import tqdm

from ament_index_python.packages import get_package_share_directory
package_share_directory = get_package_share_directory('dtefm_middle')
library_path = os.path.join(package_share_directory, 'resource/rltk')
sys.path.append(library_path)

import rl_utils

# class Trainter(Node):
    