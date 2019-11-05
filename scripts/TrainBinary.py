#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import torch
import torch.nn
import torch.nn.functional as func
from torch.utils.data import DataLoader

import rospkg
import rospy

from PolicyModelBinary import Conv3DPolicyBinary
from WaypointPolicyDataset import WaypointPolicyDataset
from WaypointDecisionDataset import WaypointDecisionDataset

import matplotlib.pyplot as pyplot

def train_iteration(model_version=0, filters=32, lr_decay=False, dropout_p=0.5, max_epochs=100, learning_rate=.00001,
                    voxel_only=False, data_only=False, time_window=90):
    use_cuda = torch.cuda.is_available()
    if use_cuda:
        print('Found cuda, using gpu.')
    else:
        print('Could not find cuda, using cpu.')
    device = torch.device('cuda:0' if use_cuda else 'cpu')  # change to 'cuda:0' with graphics card

    # num_classes = 32
    num_classes = 2

    # Training data
    datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/training32/trajectories/'
    csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/training32/all_samples_extended.csv'
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
    sampler = torch.utils.data.sampler.WeightedRandomSampler(sample_weight_tensor, len(sample_weight_tensor), replacement=True)

    # TODO: set a lower number of samples so that the weighted sampler without replacement samples balanced classes
    num_samples = 2*min(class_count)  # Samples to use for each batch (cover all of the least-seen class each epoch)

    # training_loader = DataLoader(training_set, batch_size=4, shuffle=True)
    training_loader = DataLoader(training_set, batch_size=32, shuffle=False, sampler=sampler)

    # # Validation data
    val_datapath = rospkg.RosPack().get_path('waypoint_planner') + '/data/testing32/trajectories/'
    val_csvpath = rospkg.RosPack().get_path('waypoint_planner') + '/data/testing32/all_samples.csv'
    print('Creating validation dataset from:', val_csvpath)

    validation_set = WaypointDecisionDataset(csv_file=val_csvpath, root_dir=val_datapath)
    validation_loader = DataLoader(validation_set, batch_size=32, shuffle=False)
    print('Validation set created.')

    # Stats
    training_loss = []
    validation_loss = []
    training_classification_rate = []
    validation_classification_rate = []
    training_true_rate = []
    training_false_rate = []
    validation_true_rate = []
    validation_false_rate = []
    training_f1_score = []
    validation_f1_score = []

    print('Creating model...')
    model = Conv3DPolicyBinary(model_version, filters, dropout_p=dropout_p, voxel_only=voxel_only, data_only=data_only,
                               time_window=time_window)
    loss_fn = torch.nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    if lr_decay:
        rate_lambda = lambda epoch: 0.975 ** epoch
        scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=rate_lambda)

    model = model.to(device)
    for epoch in range(max_epochs):
        print('Starting epoch', epoch)

        # Training
        model.train()
        loss_count = 0
        total_loss = 0
        total_correct = 0
        total = 0
        false_correct = 0
        true_correct = 0
        total_false = 0
        total_true = 0
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
            total += y.size(0)
            total_correct += (predicted == y).sum().item()

            false_indices = (y == 0)
            true_indices = (y == 1)
            false_predictions = (predicted == 0)
            true_predictions = (predicted == 1)

            false_correct += ((y + predicted) == 0).sum().item()
            true_correct += ((y + predicted) == 2).sum().item()
            total_false += (y == 0).sum().item()
            total_true += (y == 1).sum().item()

            if loss_count > num_samples:
                break

        true_positive = true_correct
        false_positive = total_false - false_correct
        false_negative = total_true - true_correct
        if true_positive + false_positive == 0:
            precision = 0
        else:
            precision = true_positive/(true_positive + false_positive)
        if true_positive + false_negative == 0:
            recall = 0
        else:
            recall = true_positive/(true_positive + false_negative)
        if precision + recall == 0:
            f1_score = 0
        else:
            f1_score = 2*(precision*recall)/(precision + recall)
        training_f1_score.append(f1_score)

        training_loss.append(total_loss/loss_count)
        training_classification_rate.append(total_correct/total)
        training_true_rate.append(true_correct/total_true)
        training_false_rate.append(false_correct/total_false)
        print('Epoch:', epoch, '- training loss:', training_loss[-1])
        print('\tClassification rate:', training_classification_rate[-1], '(out of', total, 'samples)')
        print('\t\tTrue rate:', training_true_rate[-1], '(of', total_true, 'samples)')
        print('\t\tFalse rate:', training_false_rate[-1], '(of', total_false, 'samples)')
        print('\tF1 score:', training_f1_score[-1])
        if training_true_rate[-1] < 0.01 or training_false_rate[-1] < 0.01:
            print('Bad model initialization, returning training failure...')
            return False, []

        # Validation
        model.eval()
        loss_count = 0
        total_loss = 0
        total_correct = 0
        total = 0
        false_correct = 0
        true_correct = 0
        total_false = 0
        total_true = 0
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
                total += y.size(0)
                total_correct += (predicted == y).sum().item()

                false_indices = (y == 0)
                true_indices = (y == 1)
                false_predictions = (predicted == 0)
                true_predictions = (predicted == 1)

                false_correct += ((y + predicted) == 0).sum().item()
                true_correct += ((y + predicted) == 2).sum().item()
                total_false += (y == 0).sum().item()
                total_true += (y == 1).sum().item()

            true_positive = true_correct
            false_positive = total_false - false_correct
            false_negative = total_true - true_correct
            if true_positive + false_positive == 0:
                precision = 0
            else:
                precision = true_positive/(true_positive + false_positive)
            if true_positive + false_negative == 0:
                recall = 0
            else:
                recall = true_positive/(true_positive + false_negative)
            if precision + recall == 0:
                f1_score = 0
            else:
                f1_score = 2*(precision*recall)/(precision + recall)
            validation_f1_score.append(f1_score)

            validation_loss.append(total_loss/loss_count)
            validation_classification_rate.append(total_correct/total)
            validation_true_rate.append(true_correct/total_true)
            validation_false_rate.append(false_correct/total_false)
            print('Epoch:', epoch, '- validation loss:', validation_loss[len(training_loss) - 1])
            print('\tClassification rate:', validation_classification_rate[-1], '(out of', total, 'samples)')
            print('\t\tTrue rate:', validation_true_rate[-1], '(of', total_true, 'samples)')
            print('\t\tFalse rate:', validation_false_rate[-1], '(of', total_false, 'samples)')
            print('\tF1 score:', validation_f1_score[-1])

        if epoch % 2 == 0:
            torch.save(model.state_dict(), 'model-epoch_' + str(epoch) + '-rate_' + str(validation_classification_rate[-1]) + '.pt')

        if lr_decay:
            scheduler.step()
            
    return True, [training_loss, validation_loss, training_classification_rate, validation_classification_rate,
                  training_true_rate, training_false_rate, validation_true_rate, validation_false_rate,
                  training_f1_score, validation_f1_score]

def plot_results(training_loss, validation_loss, training_classification_rate, validation_classification_rate,
                 training_true_rate, training_false_rate, validation_true_rate, validation_false_rate,
                 training_f1_score, validation_f1_score, title='Model 0'):
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


    training_average_rate = [0]*len(training_true_rate)
    validation_average_rate = [0]*len(validation_true_rate)
    for i in range(len(training_average_rate)):
        training_average_rate[i] = (training_true_rate[i] + training_false_rate[i])/2.0
        validation_average_rate[i] = (validation_true_rate[i] + validation_false_rate[i])/2.0
    pyplot.figure()
    line1, = pyplot.plot(epochs, training_true_rate, color='r', label='training classification rate (move)')
    line2, = pyplot.plot(epochs, training_false_rate, color='m', label='training classification rate (stay)')
    line3, = pyplot.plot(epochs, validation_true_rate, color='b', label='validation classification rate (move)')
    line4, = pyplot.plot(epochs, validation_false_rate, color='c', label='validation classification rate (stay)')
    line5, = pyplot.plot(epochs, training_average_rate, color='y', label='average training rate across classes')
    line6, = pyplot.plot(epochs, validation_average_rate, color='k', label='average validation rate across classes')
    pyplot.xlabel('epoch')
    pyplot.ylabel('classification rate')
    pyplot.axis([0, len(training_loss), 0, 1])
    pyplot.title(title)
    pyplot.legend(handles=[line1, line2, line3, line4, line5, line6], loc='lower right')
    pyplot.savefig(title + '-2.png')

    pyplot.figure()
    line1, = pyplot.plot(epochs, training_f1_score, color='r', label='training F1 score')
    line2, = pyplot.plot(epochs, validation_f1_score, color='b', label='validation F1 score')
    pyplot.xlabel('epoch')
    pyplot.ylabel('F1 score')
    pyplot.axis([0, len(training_loss), 0, 1])
    pyplot.title(title)
    pyplot.legend(handles=[line1, line2], loc='lower right')
    pyplot.savefig(title + '-3.png')

runs = 1
trial_epochs = 300
results = [[[] for i in range(10)] for j in range(runs)]

for run in range(runs):
    print('******************************************************')
    print('|  Run:', run)
    print('******************************************************')

    attempts = 0
    trained = False
    while (not trained) and (attempts < 3):
        trained, results[run] = train_iteration(model_version=5, filters=64, lr_decay=False, dropout_p=0.25,
                                                max_epochs=trial_epochs, learning_rate=0.00001)
        attempts += 1
        if not trained:
            continue
        print('Saving plots...')
        plot_results(results[run][0], results[run][1], results[run][2], results[run][3], results[run][4],
                     results[run][5], results[run][6], results[run][7], results[run][8], results[run][9],
                     'Model ' + str(run) + 'final-binary')
