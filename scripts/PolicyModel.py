#!/usr/bin/env python

from __future__ import print_function

import torch
import torch.nn as nn

class Conv3DPolicy(nn.Module):

    def __init__(self, model_version=0, filters=32, dropout_p=0.5, voxel_only=False, data_only=False, time_window=90,
                 embedding_model_version=0, num_classes=32):
        super(Conv3DPolicy, self).__init__()

        self.model_version = model_version
        self.embedding_model_version = embedding_model_version
        self.voxel_only = voxel_only
        self.data_only = data_only
        self.time_window = time_window

        if voxel_only:
            print('Creating model with voxel stream only.')
        elif data_only:
            print('Creating model with data stream only.')

        # 3D CNN for position/rotation voxel map
        self.image_conv1 = nn.Sequential(
                # nn.Conv3d(2, filters, kernel_size=3, stride=1, padding=1),
                nn.Conv3d(2, filters, kernel_size=5, stride=1, padding=2),
                nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                nn.Dropout(p=dropout_p),
                nn.MaxPool3d(3)
            )

        self.image_conv1a = nn.Sequential(
                nn.Conv3d(filters, filters, kernel_size=3, stride=1),
                # nn.Conv3d(filters, filters, kernel_size=5, stride=1, padding=1),
                nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                nn.Dropout(p=dropout_p)
            )

        self.image_conv1b = nn.Sequential(
                nn.Conv3d(filters, filters, kernel_size=3, stride=1),
                # nn.Conv3d(filters, filters, kernel_size=5, stride=1, padding=1),
                nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                nn.Dropout(p=dropout_p)
            )

        self.image_conv2 = nn.Sequential(
                nn.Conv3d(filters, filters, kernel_size=3, stride=1),
                nn.LeakyReLU(0.1),  # defaults to .01, using parameter reported from VoxNet
                nn.Dropout(p=dropout_p),
                nn.MaxPool3d(2)
                # nn.Dropout(p=dropout_p)
            )

        if model_version == 0:
            volume_dim = 6
        elif model_version == 1 or model_version == 3:
            volume_dim = 4
        elif model_version == 4:
            volume_dim = 5
        else:
            volume_dim = 6
        self.image_fc1 = nn.Sequential(
            nn.Linear(filters*volume_dim*volume_dim*volume_dim, 128),
            nn.LeakyReLU(0.1),
            nn.Dropout(p=dropout_p)
        )

        if voxel_only:
            # self.image_fc2 = nn.Sequential(
            #     nn.Linear(128, 2),
            #     nn.Softmax(dim=1)
            # )
            self.image_fc2 = nn.Sequential(
                nn.Linear(128, num_classes)
            )

        # Fully-connected network for data vector
        self.data_fc1 = nn.Sequential(
            nn.Linear(8, 64),
            nn.LeakyReLU(0.1)
        )

        self.data_fc2 = nn.Sequential(
            nn.Linear(128, 128),
            nn.LeakyReLU(0.1),
            nn.Dropout(p=dropout_p)
        )

        if data_only:
            # self.data_fcn = nn.Sequential(
            #     nn.Linear(128, 2),
            #     nn.Softmax(dim=1)
            # )
            self.data_fcn = nn.Sequential(
                nn.Linear(128, num_classes)
            )

        # Embeddings for selected data
        self.perch_embedding = nn.Embedding(2, 8)
        self.time_embedding = nn.Embedding(time_window + 1, 8)
        self.c0_embedding = nn.Embedding(91, 16)
        self.c1_embedding = nn.Embedding(91, 16)
        self.c2_embedding = nn.Embedding(91, 16)

        self.embedding_data_fc1 = nn.Sequential(
            nn.Linear(64, 64),
            nn.LeakyReLU(0.1),
            nn.Dropout(p=dropout_p)
        )

        self.embedding_data_fc1a = nn.Sequential(
            nn.Linear(64, 64),
            nn.LeakyReLU(0.1),
            nn.Dropout(p=dropout_p)
        )

        # Late fusion and classification
        self.fusion_fc1 = nn.Sequential(
            nn.Linear(128+128, 128),
            nn.LeakyReLU(0.1),
            nn.Dropout(p=dropout_p)
        )

        self.fusion_fc1a = nn.Sequential(
            nn.Linear(128, 128),
            nn.LeakyReLU(0.1),
            nn.Dropout(p=dropout_p)
        )

        # self.fusion_fc2 = nn.Sequential(
        #     nn.Linear(128, 2),
        #     nn.Softmax(dim=1)
        # )
        self.fusion_fc2 = nn.Sequential(
            nn.Linear(128, num_classes)
        )

        for m in self.modules():
            if isinstance(m, nn.Conv3d) or isinstance(m, nn.Linear):
                torch.nn.init.xavier_uniform_(m.weight, torch.nn.init.calculate_gain('leaky_relu', .1))

    def forward(self, voxel_map, data_vector):
        # if not self.data_only:
        x1 = self.image_conv1(voxel_map)
        if self.model_version != 1 and self.model_version != 3:
            x1 = self.image_conv1a(x1)
        if self.model_version == 4:
            x1 = self.image_conv1b(x1)
        x1 = self.image_conv2(x1)
        # print('image conv size final', x1.size())
        x1 = x1.view(x1.size(0), -1)
        x1 = self.image_fc1(x1)
        if self.voxel_only:
            x = self.image_fc2(x1)
            return x

        x2 = self.data_fc1(data_vector)
        xe1 = self.perch_embedding(data_vector[:, 6:7].long()).squeeze(dim=1)
        xe2 = self.time_embedding(self.time_to_vocab(data_vector[:, 7:])).squeeze(dim=1)
        xe3 = self.c0_embedding(self.cost_to_vocab(data_vector[:, 0:1])).squeeze(dim=1)
        xe4 = self.c1_embedding(self.cost_to_vocab(data_vector[:, 1:2])).squeeze(dim=1)
        xe5 = self.c2_embedding(self.cost_to_vocab(data_vector[:, 2:3])).squeeze(dim=1)
        xe = torch.cat((xe1, xe2, xe3, xe4, xe5), dim=1)
        xe = self.embedding_data_fc1(xe)
        x2 = torch.cat((x2, xe), dim=1)
        x2 = self.data_fc2(x2)
        if self.data_only:
            x = self.data_fcn(x2)
            return x

        x = torch.cat((x1, x2), dim=1)
        x = self.fusion_fc1(x)
        x = self.fusion_fc1a(x)
        x = self.fusion_fc2(x)

        return x

    def time_to_vocab(self, time):
        time_vocab = torch.clamp(time, min=0.0, max=self.time_window)
        return time_vocab.long()

    def cost_to_vocab(self, cost):
        cost_vocab = torch.clamp(cost/2.0, min=0.0, max=90.0)
        return cost_vocab.long()

# model = Conv3DPolicy(model_version=0, filters=64, voxel_only=True)
# batch_size = 4
# voxel_map = torch.randn(batch_size, 2, 32, 32, 32)
# # data_vector = torch.randn(batch_size, 8)
# data_vector = torch.FloatTensor([[1, 1, 1, 1, 1, 1, 1, 1],[0,0,0,0,0,0,0,0],[1,0,0,1,1,1,0,0],[0,0,0,1,0,1,0,1]])
#
# output = model(voxel_map, data_vector)
# print('output size: ', output.size())
# print(output)