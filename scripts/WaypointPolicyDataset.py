#!/usr/bin/env python

from __future__ import print_function

import os
import torch
from torch.utils.data import Dataset
import pandas as pd

class WaypointPolicyDataset(Dataset):
    """Waypoint policy dataset."""

    def __init__(self, csv_file, root_dir, transform=None):
        self.csv = pd.read_csv(csv_file, header=None)
        self.root_dir = root_dir

    def __len__(self):
        return len(self.csv)

    def __getitem__(self, idx):
        label = self.csv.iloc[idx, 0]
        label -= 1  # we got rid of the "observe" samples

        file_base = self.csv.iloc[idx, 1]
        pos_img_tensor = torch.load(os.path.join(self.root_dir, file_base + '-pos.pt'))
        rot_img_tensor = torch.load(os.path.join(self.root_dir, file_base + '-rot.pt'))
        data_tensor = torch.load(os.path.join(self.root_dir, file_base + '-data.pt'))

        voxel_map = torch.cat([pos_img_tensor[None, :, :, :], rot_img_tensor[None, :, :, :]], dim=0)

        sample = {'voxel_map': voxel_map, 'data_vector': data_tensor}

        return (sample, label)
