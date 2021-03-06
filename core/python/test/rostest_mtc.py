#! /usr/bin/env python

from __future__ import print_function
import unittest
import rostest
from moveit.python_tools import roscpp_init
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
import rospy


class PyGenerator(core.Generator):
    def __init__(self, name="python generator"):
        core.Generator.__init__(self, name)
        self.num = 5

    # def init(self, robot):
    #     print("INIT")
    #     core.Generator.init(self, robot)

    def reset(self):
        print('RESET')
        #core.Generator.reset(self)

    def canCompute(self):
        return self.num > 0

    def compute(self):
        self.num -= 1
        print("compute called")


class Test(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        pass

    @classmethod
    def tearDown(self):
        pass

    def test_MoveRelative(self):
        task = core.Task()
        task.add(stages.CurrentState("current"))
        move = stages.MoveRelative("move", core.JointInterpolationPlanner())
        move.group = self.PLANNING_GROUP
        move.setDirection({"joint_1" : 0.2, "joint_2" : 0.4})
        task.add(move)

        task.enableIntrospection()
        task.init()
        task.plan()

        self.assertEqual(len(task.solutions), 1)
        for s in task.solutions:
            print(s)
        s = task.solutions[0]
        task.execute(s)

    def test_Merger(self):
        cartesian = core.CartesianPath()
        def createDisplacement(group, displacement):
            move = stages.MoveRelative("displace", cartesian)
            move.group = group
            move.ik_frame = PoseStamped(header = Header(frame_id = "tool0"))
            dir = Vector3Stamped(
                header = Header(frame_id = "base_link"),
                vector = Vector3(*displacement)
            )
            move.setDirection(dir)
            move.restrictDirection(stages.MoveRelative.Direction.FORWARD)
            return move

        task = core.Task()
        task.add(stages.CurrentState("current"))
        merger = core.Merger("merger")
        merger.insert(createDisplacement(self.PLANNING_GROUP, [-0.2, 0, 0]))
        merger.insert(createDisplacement(self.PLANNING_GROUP, [ 0.2, 0, 0]))
        task.add(merger)

        task.enableIntrospection()
        task.init()
        if task.plan():
            task.publish(task.solutions[0])

    def test_PyGenerator(self):
        task = core.Task()
        keep_alive = PyGenerator()
        task.add(keep_alive)
        task.plan()


if __name__ == '__main__':
    roscpp_init("test_mtc")
    unittest.main()
