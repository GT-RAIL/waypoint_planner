#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import torch
import torch.nn
import torch.nn.functional as func
from torch.utils.data import DataLoader

from sklearn.metrics import f1_score
from sklearn.metrics import balanced_accuracy_score

import rospkg
import rospy

from PolicyModel import Conv3DPolicy
from WaypointPolicyDataset import WaypointPolicyDataset

import matplotlib.pyplot as pyplot

def train_iteration(model_version=0, filters=32, lr_decay=False, dropout_p=0.5, max_epochs=100, learning_rate=.00001,
                    voxel_only=False, data_only=False, time_window=90):
    use_cuda = torch.cuda.is_available()
    if use_cuda:
        print('Found cuda, using gpu.')
    else:
        print('Could not find cuda, using cpu.')
    device = torch.device('cuda:0' if use_cuda else 'cpu')  # change to 'cuda:0' with graphics card
    cpu_device = torch.device('cpu')

    # num_classes = 32
    # num_classes = 31
    num_classes = 30
    # num_classes = 29

    # Training data
    datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/training32/trajectories/'
    csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/training32/action_samples_extended_no_unperch.csv'
    print('Creating training dataset from:', csvpath)

    training_set = WaypointPolicyDataset(csv_file=csvpath, root_dir=datapath)
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
    sampler = torch.utils.data.sampler.WeightedRandomSampler(sample_weight_tensor, len(sample_weight_tensor), replacement=True)

    # TODO: set a lower number of samples so that the weighted sampler without replacement samples balanced classes
    num_samples = num_classes*min(class_count)  # Samples to use for each batch (cover all of the least-seen class each epoch)

    # training_loader = DataLoader(training_set, batch_size=4, shuffle=True)
    training_loader = DataLoader(training_set, batch_size=32, shuffle=False, sampler=sampler)

    # # Validation data
    val_datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/testing32/trajectories/'
    val_csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/testing32/action_samples_no_unperch.csv'
    print('Creating validation dataset from:', val_csvpath)

    validation_set = WaypointPolicyDataset(csv_file=val_csvpath, root_dir=val_datapath)
    validation_loader = DataLoader(validation_set, batch_size=32, shuffle=False)
    print('Validation set created.')

    val_class_count = [0]*num_classes
    idx = 0
    for item, label in validation_set:
        val_class_count[label] += 1
        idx += 1
    class_weight = [0]*num_classes
    total_samples = float(sum(val_class_count))
    print('Counts:', val_class_count)

    # Stats
    training_loss = []
    validation_loss = []
    training_classification_rate = []
    validation_classification_rate = []
    training_f1_score = []
    validation_f1_score = []
    training_f1_score_weighted = []
    validation_f1_score_weighted = []

    print('Creating model...')
    model = Conv3DPolicy(model_version, filters, dropout_p=dropout_p, voxel_only=voxel_only, data_only=data_only,
                               time_window=time_window, num_classes=num_classes)
    loss_fn = torch.nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    if lr_decay:
        rate_lambda = lambda epoch: 0.99 ** epoch
        scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=rate_lambda)

    model = model.to(device)
    for epoch in range(max_epochs):
        print('Starting epoch', epoch)

        # Training
        model.train()
        loss_count = 0
        total_loss = 0
        y_list = []
        y_pred_list = []

        for x, y in training_loader:
            x1 = x['voxel_map']
            x2 = x['data_vector']

            x1, x2, y = x1.to(device), x2.to(device), y.to(device)

            y_pred = model(x1, x2)

            loss = loss_fn(y_pred, y)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # update stats
            y_pred = func.softmax(y_pred, dim=1)

            loss_count += x1.size()[0]
            total_loss += loss.item()

            _, predicted = torch.max(y_pred.data, 1)

            y_list.extend(y.to(cpu_device).tolist())
            y_pred_list.extend(predicted.to(cpu_device).tolist())

            if loss_count > num_samples:
                break

        training_loss.append(total_loss/loss_count)
        training_f1_score.append(f1_score(y_list, y_pred_list, average='macro'))
        training_f1_score_weighted.append(f1_score(y_list, y_pred_list, average='weighted'))
        training_classification_rate.append(balanced_accuracy_score(y_list, y_pred_list))

        print('Epoch:', epoch, '- training loss:', training_loss[-1])
        print('\tClassification rate:', training_classification_rate[-1])
        print('\tMacro F1 score:', training_f1_score[-1])
        print('\tWeighted F1 score:', training_f1_score_weighted[-1])

        # Validation
        model.eval()
        loss_count = 0
        total_loss = 0
        y_list = []
        y_pred_list = []

        with torch.no_grad():
            for x, y in validation_loader:
                x1 = x['voxel_map']
                x2 = x['data_vector']

                x1, x2, y = x1.to(device), x2.to(device), y.to(device)

                y_pred = model(x1, x2)

                loss = loss_fn(y_pred, y)

                # update stats
                y_pred = func.softmax(y_pred, dim=1)

                loss_count += x1.size()[0]
                total_loss += loss.item()

                _, predicted = torch.max(y_pred.data, 1)

                y_list.extend(y.to(cpu_device).tolist())
                y_pred_list.extend(predicted.to(cpu_device).tolist())

            validation_loss.append(total_loss/loss_count)
            validation_f1_score.append(f1_score(y_list, y_pred_list, average='macro'))
            validation_f1_score_weighted.append(f1_score(y_list, y_pred_list, average='weighted'))
            validation_classification_rate.append(balanced_accuracy_score(y_list, y_pred_list))

            print('Epoch:', epoch, '- training loss:', validation_loss[-1])
            print('\tClassification rate:', validation_classification_rate[-1])
            print('\tMacro F1 score:', validation_f1_score[-1])
            print('\tWeighted F1 score:', validation_f1_score_weighted[-1])

        if epoch % 10 == 0:
            torch.save(model.state_dict(), 'model-epoch_' + str(epoch) + '-rate_' + str(validation_classification_rate[-1]) + '.pt')

        if lr_decay:
            scheduler.step()

    return True, [training_loss, validation_loss, training_classification_rate, validation_classification_rate,
                  training_f1_score, validation_f1_score, training_f1_score_weighted, validation_f1_score_weighted]

def plot_results(training_loss, validation_loss, training_classification_rate, validation_classification_rate,
                 training_f1_score, validation_f1_score, training_f1_score_weighted, validation_f1_score_weighted,
                 title='Model 0'):
    epochs = range(len(training_loss))

    pyplot.figure()
    line1, = pyplot.plot(epochs, training_loss, color='r', label='training loss')
    line2, = pyplot.plot(epochs, validation_loss, color='b', label='validation loss')
    pyplot.axis([0, len(training_loss), 0, .3])
    pyplot.xlabel('epoch')
    pyplot.ylabel('loss')
    pyplot.title(title)
    pyplot.legend(handles=[line1, line2], loc='lower right')
    pyplot.savefig(title + '-0.png')

    pyplot.figure()
    line1, = pyplot.plot(epochs, training_classification_rate, color='r', label='training classification rate')
    line2, = pyplot.plot(epochs, validation_classification_rate, color='b', label='validation classification rate')
    pyplot.xlabel('epoch')
    pyplot.ylabel('classification rate')
    pyplot.axis([0, len(training_loss), 0, 1])
    pyplot.title(title)
    pyplot.legend(handles=[line1, line2], loc='lower right')
    pyplot.savefig(title + '-1.png')

    pyplot.figure()
    line1, = pyplot.plot(epochs, training_f1_score, color='r', label='training F1 score (macro)')
    line2, = pyplot.plot(epochs, validation_f1_score, color='b', label='validation F1 score (macro)')
    line3, = pyplot.plot(epochs, training_f1_score_weighted, color='m', label='training F1 score (weighted)')
    line4, = pyplot.plot(epochs, validation_f1_score_weighted, color='c', label='validation F1 score (weighted)')
    pyplot.xlabel('epoch')
    pyplot.ylabel('F1 score')
    pyplot.axis([0, len(training_loss), 0, 1])
    pyplot.title(title)
    pyplot.legend(handles=[line1, line2, line3, line4], loc='lower right')
    pyplot.savefig(title + '-2.png')

runs = 1
trial_epochs = 300
results = [[[] for i in range(8)] for j in range(runs)]

for run in range(runs):
    print('******************************************************')
    print('|  Run:', run)
    print('******************************************************')

    # print('### Learning rate: .001 ###')
    attempts = 0
    trained = False
    while (not trained) and (attempts < 3):
        trained, results[run] = train_iteration(model_version=1, filters=64, lr_decay=False, dropout_p=0.125,
                                                max_epochs=trial_epochs, learning_rate=0.0005)

        print('Saving plots...')
        plot_results(results[run][0], results[run][1], results[run][2], results[run][3], results[run][4],
                     results[run][5], results[run][6], results[run][7],
                     'Policy Model ' + str(run) + 'final-model')
