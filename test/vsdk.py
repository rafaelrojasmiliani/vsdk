"""
"""
import numpy as np
from matplotlib import pyplot as plt
import sympy as sp
import unittest
from gsplines.gspline import cSplineCalc
from gsplines.distfunction import cDistFunction
from gsplines.basis1010 import cBasis1010

import functools
import traceback
import sys
import pdb


def debug_on(*exceptions):
    if not exceptions:
        exceptions = (Exception, )

    def decorator(f):
        @functools.wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except exceptions:
                info = sys.exc_info()
                traceback.print_exception(*info)
                pdb.post_mortem(info[2])

        return wrapper

    return decorator




class cMyTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(cMyTest, self).__init__(*args, **kwargs)
        np.set_printoptions(linewidth=500, precision=4)

    @debug_on()
    def test(self):


def main():
    unittest.main()


if __name__ == '__main__':
    main()