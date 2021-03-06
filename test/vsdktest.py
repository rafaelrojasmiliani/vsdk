"""
"""
import numpy as np
import unittest
import pdb

from vsdk import cVsdk
import functools
import traceback
import sys


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
        unittest.TestCase.__init__(self, *args, **kwargs)
        np.set_printoptions(linewidth=500, precision=4)

    @debug_on()
    def test(self):
        dk = cVsdk()
        dof = np.random.randint(2, 8)
        for i in range(dof):
            dh = np.random.randn(4)
            dk.add_link(*dh)


def main():
    unittest.main()


if __name__ == '__main__':
    main()
