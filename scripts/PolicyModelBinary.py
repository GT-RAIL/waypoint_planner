#!/usr/bin/env python

from __future__ import print_function

import torch
import torch.nn as nn

class Conv3DPolicyBinary(nn.Module):

    def __init__(self):
        super(Conv3DPolicyBinary, self).__init__()

        # 3D CNN for position/rotation voxel map
        self.image_conv1 = nn.Sequential(
            nn.Conv3d(2, 32, kernel_size=5, stride=2),
            nn.LeakyReLU(0.1)  # defaults to .01, using parameter reported from VoxNet
        )

        self.image_conv2 = nn.Sequential(
            nn.Conv3d(32, 32, kernel_size=3),
            nn.LeakyReLU(0.1),
            nn.MaxPool3d(2)
        )

        self.image_fc1 = nn.Sequential(
            nn.Linear(32*6*6*6, 128),
            nn.LeakyReLU(0.1)
        )

        # Fully-connected network for data vector
        self.data_fc1 = nn.Sequential(
            nn.Linear(8, 64),
            nn.LeakyReLU(0.1)
        )

        self.data_fc2 = nn.Sequential(
            nn.Linear(64, 128),
            nn.LeakyReLU(0.1)
        )

        # Late fusion and classification
        self.fusion_fc1 = nn.Sequential(
            nn.Linear(256, 128),
            nn.LeakyReLU(0.1)
        )

        self.fusion_fc2 = nn.Sequential(
            nn.Linear(128, 2),
            nn.Softmax(dim=1)
        )
        # self.fusion_fc2 = nn.Sequential(
        #     nn.Linear(128, 1),
        #     nn.Sigmoid()
        # )
        # torch.nn.init.xavier_uniform(self.fusion_fc2.weight, 1)

        for m in self.modules():
            if isinstance(m, nn.Conv3d) or isinstance(m, nn.Linear):
                torch.nn.init.xavier_uniform_(m.weight, torch.nn.init.calculate_gain('leaky_relu', .1))

    def forward(self, voxel_map, data_vector):
        # print('voxel map size:', voxel_map.size())
        x1 = self.image_conv1(voxel_map)
        # print('\tafter layer 1:', x1.size())
        x1 = self.image_conv2(x1)
        # print('\tafter layer 2:', x1.size())
        x1 = x1.view(x1.size(0), -1)
        # print('\t\tafter reshaping:', x1.size())
        x1 = self.image_fc1(x1)
        # print('\tafter layer 3:', x1.size())

        # print('data size: ', data_vector.size())
        x2 = self.data_fc1(data_vector)
        # print('\tafter layer 1:', x2.size())
        x2 = self.data_fc2(x2)
        # print('\tafter layer 2:', x2.size())

        x = torch.cat((x1, x2), dim=1)
        # print('Concatenated tensor size:', x.size())
        x = self.fusion_fc1(x)
        # print('\tafter layer 1:', x.size())
        x = self.fusion_fc2(x)
        # print('\tafter layer 2:', x.size())

        return x

# model = Conv3DPolicyBinary()
# batch_size = 4
# voxel_map = torch.randn(batch_size, 2, 32, 32, 32)
# data_vector = torch.randn(batch_size, 8)
#
# output = model(voxel_map, data_vector)
# print('output size: ', output.size())
# print(output)