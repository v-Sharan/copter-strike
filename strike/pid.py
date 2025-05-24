"""
pid class : implements a pid controller

"""

import time
from math import radians

class pid(object):

    def __init__(self, initial_p=0, initial_i=0, initial_d=0, initial_imax=0):
        # default config file
        self.p_gain = initial_p
        self.i_gain = initial_i
        self.d_gain = initial_d
        self.imax = abs(initial_imax)
        self.integrator = 0
        self.last_error = None
        self.last_update = time.time()

    # __str__ - print position vector as string
    def __str__(self):
        return "P:%s,I:%s,D:%s,IMAX:%s,Integrator:%s" % (
            self.p_gain,
            self.i_gain,
            self.d_gain,
            self.imax,
            self.integrator,
        )

    def get_dt(self, max_dt):
        """
        get_dt - returns time difference since last get_dt call
        """
        now = time.time()
        time_diff = now - self.last_update
        self.last_update = now
        if time_diff > max_dt:
            return 0.0
        else:
            return time_diff

    
    def get_p(self, error):
        """
        get_p - returns p term
        """
        return self.p_gain * error

    def get_i(self, error, dt):
        """
        get_i - returns i term
        """
        self.integrator = self.integrator + error * self.i_gain * dt
        self.integrator = min(self.integrator, self.imax)
        self.integrator = max(self.integrator, -self.imax)
        return self.integrator

    def get_d(self, error, dt):
        """
        get_d - returns d term
        """
        if self.last_error is None:
            self.last_error = error
        ret = (error - self.last_error) * self.d_gain * dt
        self.last_error = error
        return ret

    def get_pi(self, error, dt):
        """
        get_pi - returns p and i terms
        """
        return self.get_p(error) + self.get_i(error, dt)

    def get_pid(self, error, dt):
        """
        get_pid - returns p, i and d terms
        """
        return self.get_p(error) + self.get_i(error, dt) + self.get_d(error, dt)

    def get_integrator(self):
        """
        get_integrator - returns built up i term
        """
        return self.integrator

    def reset_I(self):
        """
        reset_I - clears I term
        """
        self.integrator = 0

    def main(self):

        # print object
        print("Test PID: %s" % self)

        # run it through a test
        for i in range(0, 100):
            result_p = test_pid.get_p(i)
            result_i = test_pid.get_i(i, 0.1)
            result_d = test_pid.get_d(i, 0.1)
            result = result_p + result_i + result_d
            print(
                "Err %s, Result: %f (P:%f, I:%f, D:%f, Int:%f)"
                % (i, result, result_p, result_i, result_d, self.get_integrator())
            )

if __name__ == "__main__":
    # create pid object P, I, D, IMAX
    xy_p = 1.0
    xy_i = 1.0
    xy_d = 1.0
    xy_imax = 10.0
    test_pid = pid(xy_p, xy_i, xy_d, radians(xy_imax))
    test_pid.main()
