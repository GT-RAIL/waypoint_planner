#!/usr/bin/env python

# ROS
from waypoint_planner.srv import SaveTensor1D, SaveTensor1DResponse
from waypoint_planner.srv import SaveTensor3D, SaveTensor3DResponse
import rospy

# pytorch
import torch


class TensorSaver:

    def __init__(self):
        """Initialize tensor saver services in a ROS node."""
        self.service1D = rospy.Service('~save_1D_tensor', SaveTensor1D, self.save1D)
        self.service3D = rospy.Service('~save_3D_tensor', SaveTensor3D, self.save3D)

    def save1D(self, req):
        """Save a 1D pytorch tensor that can be loaded in python."""
        tensor = torch.FloatTensor(req.tensor.data)
        torch.save(tensor, req.filename)
        return SaveTensor1DResponse()

    def save3D(self, req):
        """Save a 3D pytorch tensor that can be loaded in python."""
        data_list = [[[] for j in xrange(len(req.tensor.data[0].data))] for i in xrange(len(req.tensor.data))]
        for i in range(len(data_list)):
            for j in range(len(data_list[i])):
                data_list[i][j] = req.tensor.data[i].data[j].data
        tensor = torch.FloatTensor(data_list)
        torch.save(tensor, req.filename)
        return SaveTensor3DResponse()

if __name__ == '__main__':
    rospy.init_node('tensor_saver')

    tensor_saver = TensorSaver()
    print 'Ready to save tensors for python loading.'

    rospy.spin()