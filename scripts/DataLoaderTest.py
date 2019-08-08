#!/usr/bin/env python

from __future__ import print_function

import os
import torch
from torch.utils.data import DataLoader

import rospkg
import rospy

from WaypointPolicyDataset import WaypointPolicyDataset

rospy.init_node('data_loader_test')

datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/trajectories/'
csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/all_samples.csv'
print('Creating dataset from:', csvpath)
policy_dataset = WaypointPolicyDataset(csv_file=csvpath, root_dir=datapath)

dataloader = DataLoader(policy_dataset, batch_size=4, shuffle=True)

i = 0
for samples, labels in dataloader:
    print('------ New Batch ------')
    print(samples['voxel_map'].size(), samples['data_vector'].size(), 'labels:', labels)
    if i == 2:
        break
    i += 1
