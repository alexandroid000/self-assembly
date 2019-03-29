#! /usr/bin/env python

import unittest
from backend import *
from random import randint, random
import numpy as np


class TestExamples(unittest.TestCase):

    # add global stuff here
    def setUp(self):
        return

    def test_magnet(self):
        o = np.array([0.,0.])
        pt1 = np.array([random(),random()])
        pt2 = o + 2*pt1

        wire1 = Wire(o, "CCW")
        wire2 = Wire(o, "CW")
        pt1_test_ccw = wire1.force_at(pt1)
        pt2_test_ccw = wire1.force_at(pt2)
        pt1_test_cw = wire2.force_at(pt1)
        pt2_test_cw = wire2.force_at(pt2)
        np.testing.assert_almost_equal(pt1_test_ccw,
                                       rotate_vector(pt1_test_cw, np.pi),
                                       decimal=7, verbose=True)
        np.testing.assert_almost_equal(pt1_test_ccw/2.,
                                       pt2_test_ccw,
                                       decimal=7, verbose=True)

    def test_bb(self):
        oct = Simple_Polygon("oct",np.array(mk_regpoly(8, 5.)))
        min_x, max_x, min_y, max_y, bb = mk_bounding_box(oct)
        np.testing.assert_almost_equal(min_x, -5., decimal=7, verbose=True)
        np.testing.assert_almost_equal(max_x, 5., decimal=7, verbose=True)
        np.testing.assert_almost_equal(min_y, -5., decimal=7, verbose=True)
        np.testing.assert_almost_equal(max_y, 5., decimal=7, verbose=True)

#    def test_neighbors(self):
#        self.assertSequenceEqual(neighbors(0, env1), [4])
#        self.assertSequenceEqual(neighbors(10, env1), [5,14,9])
#
#    def test_envs(self):
#        self.assertTrue(check_grid_topo(env1))
#        self.assertTrue(check_grid_topo(env2))
#
