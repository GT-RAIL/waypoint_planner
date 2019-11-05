#!/usr/bin/env python

# Python
import random

# ROS
from waypoint_planner.srv import GetAction, GetActionResponse
import rospkg
import rospy

# pytorch
import torch
import torch.nn.functional as func

from PolicyModel import Conv3DPolicy, Conv3DPolicyBinary


class PolicyNode:

    def __init__(self):
        """Initialize tensor saver services in a ROS node."""
        self.gpu_device = torch.device('cuda:0' if use_cuda else 'cpu')
        self.cpu_device = torch.device('cpu')

        binary_model_path = rospkg.RosPack().get_path('waypoint_planner') + '/data/models/binary_model.pt'
        policy_model_path = rospkg.RosPack().get_path('waypoint_planner') + '/data/models/policy_model.pt'

        self.binary_model = Conv3DPolicyBinary(5, 64, dropout_p=0.25, time_window=90, num_classes=2)
        self.binary_model.load_state_dict(binary_model_path)  # TODO: set path
        self.binary_model.to(self.gpu_device)
        self.binary_model.eval()

        self.policy_model = Conv3DPolicy(1, 64, dropout_p=0.125, time_window=90, num_classes=30)
        self.policy_model.load_state_dict(policy_model_path)  # TODO: set path
        self.policy_model.to(self.gpu_device)
        self.policy_model.eval()

        self.servicePolicy = rospy.Service('~get_action', GetAction, self.evaluatePolicy)

    def evaluatePolicy(self, req):
        """Determine an action according to the policy approximating networks."""

        # convert data to pytorch tensors
        pos_data = [[[] for j in xrange(len(req.pos_tensor.data[0].data))] for i in xrange(len(req.pos_tensor.data))]
        for i in range(len(pos_tensor)):
            for j in range(len(pos_tensor[i])):
                pos_data[i][j] = req.pos_tensor.data[i].data[j].data
        pos_tensor = torch.FloatTensor(pos_data)
        rot_data = [[[] for j in xrange(len(req.rot_tensor.data[0].data))] for i in xrange(len(req.rot_tensor.data))]
        for i in range(len(rot_data)):
            for j in range(len(rot_data[i])):
                rot_data[i][j] = req.rot_tensor.data[i].data[j].data
        rot_tensor = torch.FloatTensor(rot_data)
        data_vector = torch.FloatTensor(req.data_tensor.data)
        data_vector = torch.unsqueeze(data_vector, 0)

        # create network input
        voxel_map = torch.cat([pos_tensor[None, :, :, :], rot_tensor[None, :, :, :]], dim=0)
        voxel_map = torch.unsqueeze(voxel_map, 0)
        x1 = voxel_map
        x2 = data_vector

        x1, x2 = x1.to(self.gpu_device), x2.to(self.gpu_device)

        # determine binary decision
        y = self.binary_model(x1, x2)
        y = func.softmax(y, dim=1)
        y = torch.squeeze(y, 0)
        y = y.to(self.cpu_device)

        hold_pos = True
        if req.stochastic:
            hold_pos = (selectAction(y.to_list()) == 1)
        else:
            _, predicted = torch.max(y.data, 1)
            hold_pos = (predicted.item() == 1)

        if hold_pos:
            return GetActionResponse(0)
        elif req.data_tensor.data[6] != 0:  # robot is perched and must take action, can only take unperch action
            return GetActionResponse(31)

        # determine action decision
        x1 = voxel_map
        x2 = data_vector

        x1, x2 = x1.to(self.gpu_device), x2.to(self.gpu_device)

        y = self.policy_model(x1, x2)
        y = func.softmax(y, dim=1)
        y = torch.squeeze(y, 0)
        y = y.to(self.cpu_device)

        hold_pos = True
        if req.stochastic:
            return GetActionResponse(selectAction(y.to_list()))
        else:
            _, predicted = torch.max(y.data, 1)
            return GetActionResponse(predicted.item())

    def selectAction(self, l):
        r = random.random()
        n = 0
        for i in range(len(y_list)):
            n += l[i]
            if n >= r:
                return i
        return len(l) - 1

if __name__ == '__main__':
    rospy.init_node('policy_node')

    print 'Loading models...'
    policy_node = PolicyNode()
    print 'Models loaded.  Ready to select actions.'

    rospy.spin()