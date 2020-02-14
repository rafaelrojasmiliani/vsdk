import numpy as np
import sympy as sp
from sympy import cos, sin


class cVsdkSym(object):
    ''' Direct Kinematic symbolic function generator.  Very
    symple funciton meat to solve very simple direct kinematic
    problems.
    '''

    def __init__(self):
        self.mij_ = []
        self.m0j_ = []
        self.q_ = []
        self.dim_ = 0
        self.mee = sp.eye(4)

    def add_link(self, _a, _d, _alpha, _theta):
        self.mij_.append(cDHmatrixSym(_a, _d, _alpha, _theta))
        self.m0j_.append(sp.zeros(4, 4))
        qs = sp.symbols('q[{:d}]'.format(len(self.q_)), real=True)
        self.q_.append(qs)
        self.dim_ += 1

    def setTCP(self, _x, _y, _z):
        self.mee[0:3, 3] = _x, _y, _z

    def __call__(self):

        q = self.q_[0]
        self.m0j_[0] = self.mij_[0](q)

        for i, mij in enumerate(self.mij_[1:], start=1):
            q = self.q_[i]
            self.m0j_[i] = self.m0j_[i - 1] * mij(q)

        res = self.m0j_[-1] * self.mee
        return res

    def __len__(self):
        return len(self.mij_)

    def jac(self):

        pe = self()[:3, -1]
        jac = sp.zeros(6, self.dim_)
        axis = sp.Matrix([0.0, 0.0, 1.0])
        p = sp.Matrix([0.0, 0.0, 0.0])
        for j in range(0, self.dim_):
            jac[:3, j] = axis.cross(pe - p).doit()
            jac[3:, j] = axis
            axis = self.m0j_[j][:3, 2]
            p = self.m0j_[j][:3, -1]

        return jac


class cDHmatrixSym(object):
    def __init__(self, _a, _d, _alpha, _theta):
        self.buff_ = sp.zeros(4, 4)
        self.theta_ = _theta
        self.alpha_ = _alpha
        self.a_ = _a
        self.d_ = _d
        self.cosal_ = cos(_alpha)
        self.sinal_ = sin(_alpha)

    def __call__(self, _q):
        cth = cos(self.theta_ + _q)
        sth = sin(self.theta_ + _q)
        res = sp.zeros(4, 4)
        res[0, 0] = cth
        res[0, 1] = -sth * self.cosal_
        res[0, 2] = sth * self.sinal_
        res[0, 3] = self.a_ * cth

        res[1, 0] = sth
        res[1, 1] = cth * self.cosal_
        res[1, 2] = -cth * self.sinal_
        res[1, 3] = self.a_ * sth

        res[2, 1] = self.sinal_
        res[2, 2] = self.cosal_
        res[2, 3] = self.d_

        res[3, 3] = 1
        return res
