#!/usr/bin/env python
__author__ = 'cholloway'
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rospy import Publisher, Subscriber, init_node, ROSInitException, sleep, spin
from numpy import arange
from pandas import DataFrame
from seaborn import violinplot
import matplotlib.pyplot as plt


class RealTimePlotter(object):
    """
      A demonstration of real-time plotting in the Robot Operating System
    """
    def __init__(self, model_name='razbot', robot_topic='/razbot_diff_drive_controller'):
        """
        Initializer
        :param model_name: the name of the model to track from gazebo
        :type model_name: str
        :param odom_topic: the topic name to analyze
        :type odom_topic: str
        :return:
        """
        self.model_name = model_name
        self.robot_topic = robot_topic
        # initialize our listener node in ROS
        init_node('RealTimePlotter')
        # initialize the velocity command publisher
        self.pub = Publisher(self.robot_topic+'/cmd_vel', Twist, queue_size=10)
        # initialize the subscriptions to data sources
        Subscriber(self.robot_topic+'/odom', Odometry, self.callback_odom)
        Subscriber('/gazebo/model_states', ModelStates, self.callback_model_state)
        self.odometry = None
        self.ground_truth = None
        self.errors = []
        self.curr_vel = None
        self.min_vel = 0.05 # m/s
        self.num_speeds = 6
        self.num_samples = 200
        self.max_vel = 4.0 # m/s
        self.radial_vel = 0.65 # rad/s
        self.my_mpl, self.ax = plt.subplots()


    def callback_odom(self, odom):
        """
        Add incoming odometry data to the odometry dataframe
        :param odom: the incoming odometry
        :type odom: nav_msgs.msg.Odometry
        :return:
        """
        self.odometry = odom.pose.pose.position
        try:
            # calculate error in odometry
            if self.ground_truth is None:
                return
            error = ((self.ground_truth.x - self.odometry.x)**2.0 + (self.ground_truth.y - self.odometry.y)**2.0)**0.5
            self.errors.append(error)
        except ROSInitException as e:
            print "waiting for node initialization: "+str(e)

    def callback_model_state(self, model_state):
        """
        Add the incoming model_state data to the model_state dataframe
        :param model_state: the incoming model state information
        :type model_state: nav_msgs.msg
        :return:
        """
        # find the model in the list of tracked gazebo models
        model_index = model_state.name.index(self.model_name)
        self.ground_truth = model_state.pose[model_index].position

    def run(self):
        """
        Run the experiment
        :return:
        """
        self.my_mpl.show()
        speeds = arange(self.min_vel, self.max_vel, abs(self.max_vel-self.min_vel)/self.num_speeds)
        observations = DataFrame()
        for speed in speeds:
            self.errors = []
            self.curr_vel = speed
            print "Starting on speed: "+str(speed)
            # initialize the speed
            _twist = Twist()
            _twist.linear.x = speed
            _twist.angular.z = self.radial_vel
            while len(self.errors) < self.num_samples:
                self.pub.publish(_twist)
                sleep(0.01)
            observations["%.3f" % speed] = self.errors[0:self.num_samples]
            violinplot(data=observations)
            self.ax.set_xlabel('Linear Speed (m/s)')
            self.ax.set_ylabel('Error (m)')
            self.my_mpl.canvas.draw()
        spin()

if __name__=="__main__":
    rtp = RealTimePlotter()
    rtp.run()