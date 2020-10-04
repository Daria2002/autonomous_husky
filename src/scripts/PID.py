from datetime import datetime
import rospy


class PID:
    """
        This class implements a simple PID control algorithm.
    """

    def __init__(self, kp=0, ki=0, kd=0, lim_high=float("inf"), lim_low=-float("inf")):
        """
            Initializes PID gains (proportional - kp, integral - ki, derivative - kd) and control values to zero.
        """

        # initialize gains
        self.kp = kp     # proportional gain
        self.ki = ki     # integral gain
        self.kd = kd     # derivative gain

        # initialize control values
        self.up = 0                     # P part
        self.ui = 0                     # I part
        self.ui_old = 0                 # I part from previous step
        self.ud = 0                     # D part
        self.u = 0                      # total control value
        self.lim_high = lim_high      # control value upper limit
        self.lim_low = lim_low    # control value lower limit

        # init referent control value (set-value)
        self.ref = 0

        # init measure control value
        self.meas = 0

        # init time stamp of the previous algorithm step
        self.t_old = datetime.now()

        self.dt = 0

        # init error from the previous algorithm step
        self.error_old = 0

        # flag indicates first step of the algorithm
        self.firstPass = True

        # ros message formed when create_msg method is called

    def reset(self):
        ''' Resets pid algorithm by setting all P,I,D parts to zero'''
        self.up = 0
        self.ui = 0
        self.ui_old = 0
        self.ud = 0
        self.u = 0
        self.t_old = datetime.now()

    def set_kp(self, invar):
        """ Set proportional gain. """
        self.kp = invar

    def get_kp(self):
        """Returns proportional gain"""
        return self.kp

    def set_ki(self, invar):
        """ Set integral gain. """
        self.ki = invar

    def get_ki(self):
        """Returns integral gain"""
        return self.ki

    def set_kd(self, invar):
        """ Set derivative gain. """
        self.kd = invar

    def get_kd(self):
        """Returns derivative gain"""
        return self.kd

    def set_lim_high(self, invar):
        """Set PID upper limit value"""
        self.lim_high = invar

    def get_lim_high(self):
        """Returns PID upper limit value"""
        return self.lim_high

    def set_lim_low(self, invar):
        """Set PID lower limit value"""
        self.lim_low = invar

    def get_lim_low(self):
        """Returns PID lower limit value"""
        return self.lim_low

    def compute(self, ref, meas, dt):
        '''
        Performs a PID computation and returns a control value based on
        the elapsed time (dt) and the error signal.
        :param ref: referent value
        :param meas: measured value
        :return: control value
        '''

        self.ref = ref
        self.meas = meas

        if self.firstPass:
            self.error_old = ref - meas
            self.firstPass = False
            return self.u           # initialized to zero
        else:
            self.ref = ref
            self.meas = meas
            error = ref - meas

            de = error - self.error_old                         # diff error
            self.up = self.kp * error                           # proportional term

            if self.ki == 0:
                self.ui = 0
            else:
                self.ui = self.ui_old + self.ki * error * dt    # integral term

            self.ud = self.kd * de / dt                         # derivative term

            self.u = self.up + self.ui + self.ud

            if self.u > self.lim_high:
                self.u = self.lim_high
                self.ui = self.ui_old  # antiwind up
            elif self.u < self.lim_low:
                self.u = self.lim_low
                self.ui = self.ui_old  # antiwind up

            self.ui_old = self.ui                           # save ui for next step
            #self.t_old = t                                  # save t for next step
            self.error_old = error
            return self.u

    def get_pid_values(self):
        """ Returns P, I, D components and total control value
        """
        return [self.up, self.ui, self.ud, self.u]