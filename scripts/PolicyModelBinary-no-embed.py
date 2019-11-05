#!/usr/bin/env python

from __future__ import print_function

import torch
import torch.nn as nn

class Conv3DPolicyBinary(nn.Module):

    def __init__(self, model_version=0, filters=32, dropout=False, voxel_only=False, data_only=False, time_window=90):
        super(Conv3DPolicyBinary, self).__init__()

        self.model_version = model_version
        self.voxel_only = voxel_only
        self.data_only = data_only
        self.time_window = time_window

        if voxel_only:
            print('Creating model with voxel stream only.')
        elif data_only:
            print('Creating model with data stream only.')

        # 3D CNN for position/rotation voxel map
        if dropout:
            if model_version == 0:
                self.image_conv1 = nn.Sequential(
                        nn.Conv3d(2, filters, kernel_size=3, stride=1, padding=1),
                        nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                        nn.Dropout(),
                        nn.MaxPool3d(2)
                    )
            elif model_version == 1:
                self.image_conv1 = nn.Sequential(
                        nn.Conv3d(2, filters, kernel_size=3, stride=2, padding=1),
                        nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                        nn.Dropout()
                    )

            if model_version == 0:
                self.image_conv2 = nn.Sequential(
                        nn.Conv3d(filters, filters, kernel_size=3, stride=1),
                        nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                        nn.Dropout(),
                        nn.MaxPool3d(2)
                    )
            elif model_version == 1:
                self.image_conv2 = nn.Sequential(
                        nn.Conv3d(filters, filters, kernel_size=3, stride=2, padding=1),
                        nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                        nn.Dropout()
                    )

            if model_version == 0:
                volume_dim = 7
            elif model_version == 1:
                volume_dim = 8
            self.image_fc1 = nn.Sequential(
                nn.Linear(filters*volume_dim*volume_dim*volume_dim, 128),
                nn.LeakyReLU(0.1),
                nn.Dropout()
            )
        else:
            if model_version == 0:
                self.image_conv1 = nn.Sequential(
                        nn.Conv3d(2, filters, kernel_size=3, stride=1, padding=1),
                        nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                        nn.MaxPool3d(2)
                    )
            elif model_version == 1:
                self.image_conv1 = nn.Sequential(
                        nn.Conv3d(2, filters, kernel_size=3, stride=2, padding=1),
                        nn.LeakyReLU(0.1)  # defaults to .01, using parameter reported from VoxNet
                    )

            if model_version == 0:
                self.image_conv2 = nn.Sequential(
                        nn.Conv3d(filters, filters, kernel_size=3, stride=1),
                        nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                        nn.MaxPool3d(2)
                    )
            elif model_version == 1:
                self.image_conv2 = nn.Sequential(
                        nn.Conv3d(filters, filters, kernel_size=3, stride=2, padding=1),
                        nn.LeakyReLU(0.1)  # defaults to .01, using parameter reported from VoxNet
                    )

            if model_version == 0:
                volume_dim = 7
            elif model_version == 1:
                volume_dim = 8
            self.image_fc1 = nn.Sequential(
                nn.Linear(filters*volume_dim*volume_dim*volume_dim, 128),
                nn.LeakyReLU(0.1)
            )

        if voxel_only:
            self.image_fc2 = nn.Sequential(
                nn.Linear(128, 2),
                nn.Softmax(dim=1)
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

        if data_only:
            self.data_fc3 = nn.Sequential(
                nn.Linear(128, 2),
                nn.Softmax(dim=1)
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

        for m in self.modules():
            if isinstance(m, nn.Conv3d) or isinstance(m, nn.Linear):
                torch.nn.init.xavier_uniform_(m.weight, torch.nn.init.calculate_gain('leaky_relu', .1))

    def forward(self, voxel_map, data_vector):
        # if not self.data_only:
        x1 = self.image_conv1(voxel_map)
        x1 = self.image_conv2(x1)
        x1 = x1.view(x1.size(0), -1)
        x1 = self.image_fc1(x1)
            # if self.voxel_only:
            #     x = self.image_fc2(x1)
            #     return x

        x2 = self.data_fc1(data_vector)
        x2 = self.data_fc2(x2)
        # if self.data_only:
        #     x = self.data_fc3(x2)
        #     return x

        x = torch.cat((x1, x2), dim=1)
        x = self.fusion_fc1(x)
        x = self.fusion_fc2(x)

        return x

    def time_to_vocab(self, time):
        time_vocab = torch.clamp(time, min=0.0, max=self.time_window)
        return time_vocab.long()

# model = Conv3DPolicyBinary(model_version=4, filters=32, leaky=True)
# batch_size = 4
# voxel_map = torch.randn(batch_size, 2, 32, 32, 32)
# data_vector = torch.randn(batch_size, 8)
#
# output = model(voxel_map, data_vector)
# print('output size: ', output.size())
# print(output)