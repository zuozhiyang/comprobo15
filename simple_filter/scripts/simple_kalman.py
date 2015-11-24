#!/usr/bin/env python

"""
    This script implements a Kalman filter for the system:

    x_0 ~ N(0, sigma_sq)
    x_t = x_{t-1} + w_t, w_t ~ N(0, sigma_m_sq)
    z_t = x_t + v_t, v_t ~ N(0, sigma_z_sq)
"""

import matplotlib.pyplot as plt
import rospy
from numpy import arange
from numpy.random import randn
from math import e, sqrt, pi

def get_z_t(x_true, sigma_sq_z):
    """ Sample an observation centered at x_true plus Gaussian noise
        with variance sigma_sq_z and mean 0 """
    return x_true + sqrt(sigma_sq_z)*randn()

def get_x_t(x_true, sigma_sq_m):
    """ Sample next system state as the current system state plus Gaussian
        noise with variance sigma_sq_m and mean 0 """
    return x_true + sqrt(sigma_m_sq)*randn()

def plot_pdf(mu, sigma_sq, x_true, z, graphs=None):
    """ Plot the Gaussian PDF with the specified mean (mu) and variance (sigma_sq)
        x_true is the true system state which will be plotted in blue
        z is the current observation which will be plotted in red """
    xs = arange(-5, 5, .005)
    p_of_x = [1./sqrt(2*pi*sigma_sq)*e**(-(x - mu)**2/(2*sigma_sq)) for x in xs]
    if graphs:
        graphs[0].set_ydata(p_of_x)
        graphs[1].set_xdata(x_true)
        graphs[2].set_xdata(z)
    else:
        graphs = []
        graphs.append(plt.plot(xs, p_of_x)[0])
        graphs.append(plt.plot(x_true, 0,'b.')[0])
        graphs.append(plt.plot(z, 0,'r.')[0])
        graphs[1].set_markersize(20)
        graphs[2].set_markersize(20)
        plt.ylim([0, 5])
    plt.show(False)
    return graphs

rospy.init_node('simple_kalman')

plt.ion()
T = 100

# initial beliefs
mu = 0
sigma_sq = 1

# sample system state
x_true = mu + sqrt(sigma_sq)*randn()

# motor noise
sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
# observation noise
sigma_z_sq = rospy.get_param('~sigma_z_sq', .1)
# time to pause between plots
pause_time = rospy.get_param('~pause_time', 0.5)

graphs = None
while not rospy.is_shutdown():
    # Graph new observation from the system
    z_t = get_z_t(x_true, sigma_z_sq)
    graphs = plot_pdf(mu, sigma_sq, x_true, z_t, graphs)

    # Do Kalman updates
    K_t = (sigma_sq + sigma_m_sq)/(sigma_sq + sigma_m_sq + sigma_z_sq)
    mu = mu + K_t*(z_t - mu)
    sigma_sq = (1-K_t)*(sigma_sq+sigma_m_sq)
    plt.pause(pause_time)
    graphs = plot_pdf(mu, sigma_sq, x_true, z_t, graphs)

    # sample next state
    x_true = get_x_t(x_true, sigma_m_sq)
    plt.pause(pause_time)