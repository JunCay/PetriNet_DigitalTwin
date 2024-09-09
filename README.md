# Intelligent Sorter Robot System Based on DigitalTwin and Petri-Net Reinforcement Learning
## 1. Project Overview
This project is aiming at achieving:
- A Intelligent Sorting Robot **DigitalTwin** system which matches the simulate(virtual) environment to the physcial(actual) environment of a sorter robot.
  - Simulate Environment based on  ***Unity3D***
  - Distributed Sytem structure based on ***ROS2***
- Discrete Abstract of Sorter-Task and Intelligent Scheduling
  - Discription in form of ***Petri-Net***
  - AI implementation of ***RL***

## 2. Sysyem Structure (updating)
### 2.0. Develop Environments
- **Communication**: ROS2 (humble) on WSL2 Ubuntu22.04
- **SimulateEnv**: Unity3D 2021.3.7f1 + ROS-TCP-Connector + URDF Importer
- **PhysicalENv**: EFM

![alt text](src/resources/images/StructureV2.0.jpg)

### 2.1. ROS2 Packages Description
***dtefm***: Digital Twin EFM (also sorter)
- dtefm_core
  - Description
    - core nodes and launch files
  - Contents
    - dtefm_command_**
      - publisher: command sending simulator
      - decoder: universal decoder to
      - server: command parser
      - gate: distributer
    - dtefm_ether_gate: tcp communication between PC & robot controller
    - dtefm_identity
- dtefm_middle
  - Description
    - Robot Intelligent nodes (self recognition)
  - Contents
    - dtefm_identity: system recognition on sr_state and petri-net state
    - dtefm_sr_IK_server: sr robot inverse kinematics
    - dynamic_schedule: (TBD, dynamic petri-net)
- dtefm_physical
  - Description
    - Physical Envrionment related
  - Contents
    - dtefm_ether_gate_p: (aborted) original gate to physical env only
    - dtefm_ether_sender_p: send tcp command to physical robot controller
    - dtefm_sr_state_sync_p: send tcp command requesting sr state to physical robot controller continuously
- dtefm_simulated
  - Description
    - Simulate Environment related
  - Contents
    - dtefm_ether_sender_s: send tcp command(simulated) to simulate environment
- dtefm_interfaces
  - Description
    - Customized msg/srv/action
- ros_tcp_endpoint (From https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
  - Description
    - connect ROS2 and Unity3D

### 2.2. Simulate Environment
#### 2.2.1. Main Scene
![alt text](src/resources/images/ToP.gif)

#### 2.2.2. Dynamic Petri Net Visualization
![alt text](src/resources/images/DPN_create.gif)![alt text](src/resources/images/DPN.gif)

## 3. Toolkits
### 3.1. Petri Net Tool Kit (**PNTK**)
[**PetriNetPython**](src/dtefm_middle/resource/pntk/example_nets.py)

Used in dtefm_middle

Describe and create **Timed Color Petri Net**

### 3.2. Dynamic Petri Net Viewer
[**DPNViewer**](src/dtefm_middle/resource/dpn(unity_scripts)/pn_updator.cs)

Based in Unity3d: **pn_command <-> dtefm_identity <-> ros2 msg+srv <-> unity3d**