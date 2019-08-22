#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import torch
import torch.nn
from torch.utils.data import DataLoader

import rospkg
import rospy

from PolicyModel import Conv3DPolicy
from WaypointPolicyDataset import WaypointPolicyDataset
from WaypointDecisionDataset import WaypointDecisionDataset

use_cuda = torch.cuda.is_available()
if use_cuda:
    print('Found cuda, using gpu.')
else:
    print('Could not find cuda, using cpu.')
device = torch.device('cuda:0' if use_cuda else 'cpu')  # change to 'cuda:0' with graphics card

max_epochs = 1000

# num_classes = 32
num_classes = 2

# Training data
datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/training/trajectories/'
csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/training/all_samples.csv'
print('Creating training dataset from:', csvpath)

training_set = WaypointDecisionDataset(csv_file=csvpath, root_dir=datapath)
print('Training set created.')

class_count = [0]*num_classes
idx = 0
for item, label in training_set:
    class_count[label] += 1
    idx += 1
class_weight = [0]*num_classes
total_samples = float(sum(class_count))
print('Counts:', class_count)
for i in range(num_classes):
    if class_count[i] == 0:
        class_weight[i] = 0
    else:
        class_weight[i] = total_samples/float(class_count[i])
sample_weight = [0]*len(training_set)
idx = 0
print('Class weights calculated:', class_weight)
for item, label in training_set:
    sample_weight[idx] = class_weight[label]
    idx += 1
print('Sample weights calculated.')
sample_weight_tensor = torch.DoubleTensor(sample_weight)
sampler = torch.utils.data.sampler.WeightedRandomSampler(sample_weight_tensor, len(sample_weight_tensor), replacement=False)

# TODO: set a lower number of samples so that the weighted sampler without replacement samples balanced classes
num_samples = min(class_count)  # Samples to use for each batch (cover all of the least-seen class each epoch)

# training_loader = DataLoader(training_set, batch_size=4, shuffle=True)
training_loader = DataLoader(training_set, batch_size=4, shuffle=False, sampler=sampler)

# # Validation data
val_datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/testing/trajectories/'
val_csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/testing/all_samples.csv'
print('Creating validation dataset from:', val_csvpath)

validation_set = WaypointDecisionDataset(csv_file=val_csvpath, root_dir=val_datapath)
validation_loader = DataLoader(validation_set, batch_size=4, shuffle=False)

# Stats
training_loss = []
validation_loss = []

print('Creating model...')
model = Conv3DPolicy(num_classes=num_classes)
loss_fn = torch.nn.CrossEntropyLoss()
learning_rate = .001
# optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
optimizer = torch.optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
# rate_lambda = lambda epoch: 0.95 ** epoch
# scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=rate_lambda)

model = model.to(device)
for epoch in range(max_epochs):
    print('Starting epoch', epoch)

    # Training
    # model.train()
    loss_count = 0
    total_loss = 0
    total_correct = 0
    total = 0
    for x, y in training_loader:
        x1 = x['voxel_map']
        x2 = x['data_vector']

        # print('x1 size:', x1.size())
        # print('x2 size:', x2.size())
        x1, x2, y = x1.to(device), x2.to(device), y.to(device)

        optimizer.zero_grad()

        y_pred = model(x1, x2)

        loss = loss_fn(y_pred, y)

        # optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # update stats
        loss_count += x1.size()[0]
        total_loss += loss.item()

        _, predicted = torch.max(y_pred.data, 1)
        total += y.size(0)
        total_correct += (predicted == y).sum().item()

    training_loss.append(total_loss/loss_count)
    print('Epoch:', epoch, '- training loss:', training_loss[len(training_loss) - 1])
    print('\tClassification rate:', total_correct/total, '(out of', total, 'samples)')

    # # Validation
    # # model.eval()
    # loss_count = 0
    # total_loss = 0
    # with torch.no_grad():
    #     for x, y in validation_loader:
    #         x1 = x['voxel_map']
    #         x2 = x['data_vector']
    #
    #         x1, x2, y = x1.to(device), x2.to(device), y.to(device)
    #
    #         y_pred = model(x1, x2)
    #
    #         loss = loss_fn(y_pred, y)
    #
    #         # update stats
    #         loss_count += x1.size()[0]
    #         total_loss += loss.item()
    #
    #     validation_loss.append(total_loss/loss_count)
    #     print('Epoch:', epoch, '- validation loss:', validation_loss[len(training_loss) - 1])

    # scheduler.step()
# print('Done')