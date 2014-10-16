#!/usr/bin/env python
import roslib
roslib.load_manifest('human_fall_detector')
import rospy
import std_msgs.msg
import human_fall_detector.msg
import math
import collections

Tv = 0.8
TvH = 0.03
TvWD = 0.11
TvZ = 0.65
TiH = 0.45
counter_a = counter_b = 0
activity_detection = 0

counter_falling = 0.12
counter_inactivity = 0.5
counter_activity_detection = 2.0

vH_buf = collections.deque(maxlen=9)
vWD_buf = collections.deque(maxlen=9)
vZ_buf = collections.deque(maxlen=9)
vH = 0
vWD = 0
vZ = 0

last_state = None

# http://wiki.scipy.org/Cookbook/SavitzkyGolay
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')

def dist(x, y, z=0):
    return math.sqrt(x**2 + y**2 + z**2)


import matplotlib.pyplot as plt
plt.ion()
fig = plt.figure()
state_ax = fig.add_subplot(1, 2, 1)
state_bars = state_ax.bar((0, 1, 2), (1, 1, 1), (1, 1, 1), color=['y', 'm', 'r'])
plt.plot([0, 1], [counter_falling, counter_falling], color='r')
plt.plot([1, 2], [counter_activity_detection, counter_activity_detection], color='g')
plt.plot([2, 3], [counter_inactivity, counter_inactivity], color='r')
state_ax.set_ylim([0, counter_activity_detection])
state_ax.set_xticks((0.5, 1.5, 2.5))
state_ax.set_xticklabels(('Falling', 'Settling', 'Inactivity'))

data_ax = fig.add_subplot(1, 2, 2)
data_bars = data_ax.bar((0, 1, 2), (1, 1, 1), (1, 1, 1), color=['b', 'b', 'b'])
plt.plot([0, 2], [Tv, Tv], color='b')
plt.plot([0, 2], [-Tv, -Tv], color='b')
plt.plot([0, 1], [-TvH, -TvH], color='r')
plt.plot([1, 2], [TvWD, TvWD], color='r')
plt.plot([2, 3], [-TvZ, -TvZ], color='r')
plt.plot([0, 1], [TiH, TiH], color='y')
data_ax.set_ylim([-1.5, 1.5])
data_ax.set_xticks((0.5, 1.5))
data_ax.set_xticklabels(('vH', 'vWD', 'vZ'))

def state_callback(state):
    global TvH, TvWD, TvZ, TiH
    global counter_a, counter_b
    global counter_falling, counter_inactivity
    global activity_detection

    global vH, vWD, vZ
    global vH_pub, vWD_pub, vZ_pub
    global vH_buf, vWD_buf, vZ_buf

    global last_state
    if last_state is not None:
        dt = (state.header.stamp - last_state.header.stamp).to_sec()
        if dt == 0:
            return
        vH_raw = (state.height - last_state.height) / dt
        vWD_raw = (dist(state.width, state.depth)
                   - dist(last_state.width, last_state.depth)) / dt
        vZ_raw = (state.z - last_state.z) / dt
        if abs(vH_raw) > Tv or abs(vWD_raw) > Tv:
            return


        vH_buf.append(vH_raw)
        vWD_buf.append(vWD_raw)
        vZ_buf.append(vZ_raw)

        if len(vH_buf) < 9:
            return

        import numpy as np
        vH = savitzky_golay(np.array(vH_buf), 9, 2)[len(vH_buf)-1]
        vWD = savitzky_golay(np.array(vWD_buf), 9, 2)[len(vWD_buf)-1]
        vZ = savitzky_golay(np.array(vZ_buf), 9, 2)[len(vZ_buf)-1]

        vH_pub.publish(vH)
        vWD_pub.publish(vWD)
        vZ_pub.publish(vZ)


        if (vH < -TvH) and (vWD > TvWD) and (vZ < -TvZ):
            counter_a = counter_a + dt
            if counter_a >= counter_falling:
                rospy.loginfo("Falling Detected")
                activity_detection = counter_activity_detection
        elif counter_a > 0:
            counter_a = counter_a - dt * 0.5

        if activity_detection > 0:
            if vH < TiH:
                if counter_b >= counter_inactivity:
                    rospy.loginfo("Fall Detected")
                counter_b = counter_b + dt
            elif counter_b > 0:
                counter_b = counter_b - dt
            activity_detection = activity_detection - dt
        else:
            counter_b = 0

        rospy.logdebug("State: %r, %r, %r - vH=%f, vWD=%f", activity_detection, counter_a, counter_b, vH, vWD)

    last_state = state


if __name__ == '__main__':
    rospy.init_node('human_fall_detector')
    reference_frame = rospy.get_param('~reference_frame', 'openni_depth_frame')
    rospy.Subscriber("human/state", human_fall_detector.msg.PersonState, state_callback)
    vH_pub = rospy.Publisher('human/vH', std_msgs.msg.Float64, queue_size=1)
    vWD_pub = rospy.Publisher('human/vWD', std_msgs.msg.Float64, queue_size=1)
    vZ_pub = rospy.Publisher('human/vZ', std_msgs.msg.Float64, queue_size=1)
    #rospy.spin()
    while not rospy.is_shutdown():
        state_bars[0].set_height(counter_a)
        state_bars[1].set_height(activity_detection)
        state_bars[2].set_height(counter_b)

        data_bars[0].set_height(vH)
        data_bars[1].set_height(vWD)
        data_bars[2].set_height(vZ)
        plt.draw()
        plt.pause(0.05)
