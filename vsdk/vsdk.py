import numpy as np
from math import cos, sin

class cVsdk(object):
    ''' Direct Kinematic Function.  Very symple funciton meat to
    solve very simple direct kinematic problems.
    '''
    def __init__(self):
        self.mij_ = []
        self.m0j_ = []
        self.axis_ = np.zeros((3, ))
        self.p_ = np.zeros((3, ))
        self.jac_ = np.zeros((6, 6))
        self.mee = np.eye(4)
        pass

    def add_link(self, _a, _d, _alpha, _theta):
        self.mij_.append(cDHmatrix(_a, _d, _alpha, _theta))
        self.m0j_.append(np.zeros((4, 4)))

    def set_tcp_offset(self, _x, _y, _z):
        self.mee[0:3, 3] = _x, _y, _z

    def __call__(self, _q):

        self.m0j_[0][:, :] = self.mij_[0](_q[0])

        for i, mij in enumerate(self.mij_[1:]):
            self.m0j_[i + 1][:, :] = self.m0j_[i].dot(mij(_q[i + 1]))

        res = self.m0j_[-1]*self.mee
        return res

    def __len__(self):
        return len(self.mij_)

    def jac(self, _q):
        
        pe = self(_q)[:3, -1]
        self.axis_[:] = [0.0, 0.0, 1.0]
        self.p_[:] = [0.0, 0.0, 0.0]
        for j in range(0, 6):
            self.jac_[:3, j] = np.cross(self.axis_, pe - self.p_)
            self.jac_[3:, j] = self.axis_
            self.axis_[:] = self.m0j_[j][:3, 2]
            self.p_[:] = self.m0j_[j][:3, -1]

        return self.jac_

    def vel(self, _q, _qd):
        """vel

        :param _q numpy array, joint position:
        :param _qd numpy array, joint velocity:
        :return numpy array :
        """

        jac = self.jac(_q)

        return jac.dot(_qd)


class cDHmatrix(object):
    ''' Class representing a homogeneous transformation matrix
    given by the Denavit-Hartenberg parameters.
    '''
    def __init__(self, _a, _d, _alpha, _theta):
        self.buff_ = np.zeros((4, 4))
        self.theta_ = _theta
        self.alpha_ = _alpha
        self.a_ = _a
        self.d_ = _d
        self.cosal_ = np.cos(_alpha)
        self.sinal_ = np.sin(_alpha)

    def __call__(self, _q, _buff=None):
        """__call__, returns a homogeneous transformation numpy
        matrix

        :param _q: float, scalar
        :param _buff: nympy matrix, array to fill the matrix

        Returns a numpy matrix
        """
        if _buff is None:
            _buff = self.buff_
        else:
            _buff[3, :3] = 3*[0.0]

        cth = cos(self.theta_ + _q)
        sth = sin(self.theta_ + _q)
        _buff[0, 0] = cth
        _buff[0, 1] = -sth * self.cosal_
        _buff[0, 2] = sth * self.sinal_
        _buff[0, 3] = self.a_ * cth

        _buff[1, 0] = sth
        _buff[1, 1] = cth * self.cosal_
        _buff[1, 2] = -cth * self.sinal_
        _buff[1, 3] = self.a_ * sth

        _buff[2, 1] = self.sinal_
        _buff[2, 2] = self.cosal_
        _buff[2, 3] = self.d_

        _buff[3, 3] = 1

        return _buff
