#!/usr/bin/python3.6
from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
import serial
import re
import cv2
from copy import copy

from sensor_msgs.msg import NavSatFix

from gps_library import gps_read_coordinates

class GPS():
    def __init__(self):
        # definition of serial port
        self.port="/dev/ttyUSB0"
        self.ser=serial.Serial(self.port, baudrate=9600, timeout=0.5)

        # receive coordinates flag
        self.received = False

        # coordinates
        self.lat = 0.0
        self.lng = 0.0

        # time
        self.time = None

        # previous coordinates
        self.lat_p = 0.0
        self.lng_p = 0.0

        # coordinates message
        self.msg = NavSatFix()

        # coordinates publisher
        self.gps_pub = rospy.Publisher("gps_point_lat_lng", NavSatFix, queue_size=2)

        # flag of initialization process
        self.init = False

        # set if want to plot
        self.show_plot = True

        # first set of measurements
        self.measurements = []

        # kalman filter
        self.kf1 = None

        self.kf3 = None
        self.filtered_state_means = None
        self.filtered_state_covariances = None

        self.initial_state_mean = None

        # ax for plot
        self.ax = plt.axes()


        # arrays for plot
        self.lat_array = []
        self.x_now_array = []

        # define transform matrix
        self.transition_matrix = [[1, 1, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 1],
                                  [0, 0, 0, 1]]

        # define observation matrix
        self.observation_matrix = [[1, 0, 0, 0],
                                   [0, 0, 1, 0]]

    #def get_new_coordinates(self):


    # get set of first eight measurements
    def get_first_set_measurements(self):

        while not self.init and not rospy.is_shutdown():
            print("len of measurement: ", len(self.measurements))

            newdata=self.ser.readline()
            self.received, self.lat, self.lng, self.time = gps_read_coordinates(newdata)

            if self.check_and_update_state():
                self.initial_state_mean = [self.lat, 0, self.lng, 0]
                
                self.publish_coordinates(self.lat, self.lng)

                self.measurements.append((self.lat, self.lng))

            if len(self.measurements) >= 8:
                self.init = True
            print("init")

        # set init measurements
        self.measurements = np.asarray(self.measurements)

    def check_and_update_state(self):
        if self.received and abs(self.lat - self.lat_p) > 1.0e-17 and abs(self.lat - self.lat_p) > 1.0e-17:

            self.lat_p = copy(self.lat)
            self.lng_p = copy(self.lng)
            return True

        else:
            return False


    def count_initial_filter_parameters(self):
        # first iteration of kalman filter
        self.kf1 = KalmanFilter(transition_matrices = self.transition_matrix,
                           observation_matrices = self.observation_matrix,
                           initial_state_mean = self.initial_state_mean)

        self.kf1 = self.kf1.em(self.measurements, n_iter=50)
        (smoothed_state_means, smoothed_state_covariances) = self.kf1.smooth(self.measurements)


    # initialization part
    def initialization_part(self):
        n_real_time = 3

        self.kf3 = KalmanFilter(transition_matrices = self.transition_matrix,
                                observation_matrices = self.observation_matrix,
                                initial_state_mean = self.initial_state_mean,
                                observation_covariance = self.kf1.observation_covariance,
                                em_vars=['transition_covariance', 'initial_state_covariance'])

        self.kf3 = self.kf3.em(self.measurements[:-n_real_time, :], n_iter=50)
        (self.filtered_state_means, self.filtered_state_covariances) = self.kf3.filter(self.measurements[:-n_real_time,:])


    def do_all(self):

        x_now = self.filtered_state_means
        P_now = self.filtered_state_covariances

        # main loop 
        while not rospy.is_shutdown():
            newdata=self.ser.readline()
            self.received, self.lat, self.lng, self.time = gps_read_coordinates(newdata)
            if self.check_and_update_state():
                m = [self.lat, self.lng]

                (x_next, P_next) = self.kf3.filter_update(filtered_state_mean = x_now[-1, :],
                                                          filtered_state_covariance = P_now[-1, :],
                                                          observation = m)

                print("differences: ", self.lat - x_next[0], "  ", self.lng - x_next[2])
                print("Lat: ", x_next[0], " Lng: ", x_next[2])
                
                # update array
                x_now = np.append(x_now, np.array([x_next]), axis=0)
                P_now = np.append(P_now, np.array([P_next]), axis = 0)

                # publish coordinates
                self.publish_coordinates(x_next[0], x_next[2])

                #plot
                self.plot_coordinates(x_next)




    # plot coordinates
    def plot_coordinates(self, x_next):

        # plot
        self.lat_array.append(self.lat)
        self.x_now_array.append(x_next[0])  
        t = np.arange(len(self.x_now_array))    

        if self.show_plot:

            self.ax.plot(t, self.lat_array, 'bo-', label='Original')
            self.ax.plot(t, self.x_now_array, 'rx-', label='Filtered')
            #leg = self.ax.legend();
            #self.ax.set_xlabel('time [s]')
            #self.ax.set_ylabel('latitude [°]')
            plt.draw() 
            plt.pause(0.01) #is necessary for the plot to update for some reason
        #i += 1
        #input("Press Enter to continue...")

    def publish_coordinates(self, lat, lng):
        self.msg.latitude = lat
        self.msg.longitude = lng
        self.msg.header.stamp = self.time
        self.gps_pub.publish(self.msg)



def main():
    # init ros node
    rospy.init_node('gps', anonymous = True)

    # create gps object
    gps_object = GPS()

    # get set of first eight measurements
    gps_object.get_first_set_measurements()

    # count initial parameters
    gps_object.count_initial_filter_parameters()

    # initialization part
    gps_object.initialization_part()

    # run program
    gps_object.do_all()


if __name__ == '__main__':
    main()


