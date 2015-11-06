#!/usr/bin/env python
__author__ = 'cholloway'
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rospy import Publisher, Subscriber, init_node, sleep, spin
from numpy import arange, NaN, empty
from pandas import DataFrame
from seaborn import violinplot, set_context
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


        # initialize the ROS node
        init_node('real_time_plotter')
        # initialize the velocity command publisher
        self.pub = Publisher(self.robot_topic+'/cmd_vel', Twist, queue_size=10)
        # initialize the subscriptions to data sources
        Subscriber(self.robot_topic+'/odom', Odometry, self.callback_odom)
        Subscriber('/gazebo/model_states', ModelStates, self.callback_model_state)

        self.odom_frequency = 50.0
        self.odometry = None
        self.ground_truth = None
        self.last_gt = None
        self.errors = []
        self.curr_vel = None
        self.min_vel = 0.10 # m/s
        self.num_speeds = 4
        self.num_samples = 500
        self.max_vel = 0.5 # m/s
        self.radial_vel = 0.65 # rad/s
        self.my_mpl, self.ax = plt.subplots()
        self.ax.set_xlabel('Linear Speed (m/s)')
        self.ax.set_ylabel('Error (mm)')
        self.my_mpl.show()

    def callback_odom(self, odom):
        """
        Add incoming odometry data to the self.odometry and calculate the error
        :param odom: the incoming odometry
        :type odom: nav_msgs.msg.Odometry
        """
        # make sure there is ground truth data to compare against
        if self.ground_truth is None:
            return

        if self.odometry is None:
            self.odometry = odom.pose.pose.position
        else:
            last_odom = self.odometry
            self.odometry = odom.pose.pose.position
            # calculate displacement magnitudes
            ground_truth_dp = ((self.ground_truth.x-self.last_gt.x)**2.0 + (self.ground_truth.y-self.last_gt.y)**2.0)
            odom_dp = ((self.odometry.x-last_odom.x)**2.0 + (self.odometry.y-last_odom.y)**2.0)
            self.errors.append((ground_truth_dp-odom_dp)*1000.0*self.odom_frequency)
        self.last_gt = self.ground_truth

    def callback_model_state(self, model_state):
        """
        Add the incoming model_state data to self.ground_truth
        :param model_state: the incoming model state information
        :type model_state: gazebo_msgs.msg.ModelStates
        """
        # find the model in the list of tracked gazebo models
        model_index = model_state.name.index(self.model_name)
        # grab the position of the robot model
        self.ground_truth = model_state.pose[model_index].position

    def run(self):
        """
        Run the experiment
        """
        speeds = arange(self.min_vel, self.max_vel, abs(self.max_vel-self.min_vel)/self.num_speeds)
        observations = DataFrame(empty((self.num_samples, len(speeds)))*NaN, columns=(["%.2f" % sp for sp in speeds]))
        for speed in speeds:
            self.errors = []
            self.curr_vel = speed
            # initialize the speed
            _twist = Twist()
            _twist.linear.x = speed
            _twist.angular.z = self.radial_vel
            # keep publishing that speed until we have enough samples
            while len(self.errors) < self.num_samples:
                plt.cla()
                upper = min(len(self.errors), self.num_samples)
                # copy up to self.num_samples into dataframe
                observations["%.2f" % speed][0:upper] = self.errors[0:upper]
                # plot dataframe
                self.ax.set_xlabel('Linear Speed (m/s)',fontsize=16)
                self.ax.set_ylabel('Error (mm/s)',fontsize=16)
                self.ax.set_title('Currently driving at: %.2f m/s' % speed)
                violinplot(data=observations)
                set_context("notebook", font_scale=1.5, rc={"lines.linewidth": 2.5})
                self.my_mpl.canvas.draw()
                self.pub.publish(_twist)
                sleep(0.01)
        spin()


if __name__ == "__main__":
    rtp = RealTimePlotter()
    rtp.run()