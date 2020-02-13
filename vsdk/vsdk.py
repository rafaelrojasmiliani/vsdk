import numpy as np
from math import cos, sin

class cDk(object):
    ''' Direct Kinematic Function.
        Very symple funciton meat to solve very simple direct kinematic
        problems.
    '''
    def __init__(self):
        self.mij_ = []
        self.m0j_ = []
        self.axis_ = np.zeros((3, ))
        self.p_ = np.zeros((3, ))
        self.jac_ = np.zeros((6, 6))
        pass

    def add_link(self, _a, _d, _alpha, _theta):
        self.mij_.append(cDHmatrix(_a, _d, _alpha, _theta))
        self.m0j_.append(np.zeros((4, 4)))

    def __call__(self, q):

        self.m0j_[0][:, :] = self.mij_[0](q[0])

        for i, mij in enumerate(self.mij_[1:]):
            self.m0j_[i + 1][:, :] = self.m0j_[i].dot(mij(q[i + 1]))

        return self.m0j_[-1]

    def __len__(self):
        return len(self.mij_)

    def jac(self, q):
        
        pe = self(q)[:3, -1]
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
    def __init__(self, _a, _d, _alpha, _theta):
        self.buff_ = np.zeros((4, 4))
        self.theta_ = _theta
        self.alpha_ = _alpha
        self.a_ = _a
        self.d_ = _d
        self.cosal_ = np.cos(_alpha)
        self.sinal_ = np.sin(_alpha)

    def __call__(self, q):
        cth = cos(self.theta_ + q)
        sth = sin(self.theta_ + q)
        self.buff_[0, 0] = cth
        self.buff_[0, 1] = -sth * self.cosal_
        self.buff_[0, 2] = sth * self.sinal_
        self.buff_[0, 3] = self.a_ * cth

        self.buff_[1, 0] = sth
        self.buff_[1, 1] = cth * self.cosal_
        self.buff_[1, 2] = -cth * self.sinal_
        self.buff_[1, 3] = self.a_ * sth

        self.buff_[2, 1] = self.sinal_
        self.buff_[2, 2] = self.cosal_
        self.buff_[2, 3] = self.d_

        self.buff_[3, 3] = 1
        return self.buff_
