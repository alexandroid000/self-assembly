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
#    def test_simple_step_gen(self):
#        ns = neighbors(3, env2)
#        self.assertSequenceEqual(ns, [5,2])
#        p1 = gridworld_step_prob(3,5, types2[0], env2)
#        self.assertEqual(p1, 0.2)
#        p2 = gridworld_step_prob(3,2, types2[1], env2)
#        self.assertEqual(p2, 0.3)
#
#    def test_transitions(self):
#        output = np.array([[[[0.33333333, 0.33333333, 0.        , 0.        , 0.33333333, 0.  ],
#                           [0.25      , 0.25      , 0.25      , 0.        , 0.25 , 0.      ],
#                           [0.        , 0.33333333, 0.33333333, 0.33333333, 0. , 0.        ],
#                           [0. , 0.  , 0.33333333, 0.33333333, 0.        , 0.33333333],
#                           [0.33333333, 0.33333333, 0.        , 0.        , 0.33333333, 0.        ],
#                           [0.        , 0.        , 0.        , 0.5       , 0.        , 0.5       ]]],
#                       [[[0.29411765, 0.35294118, 0. , 0.        , 0.35294118, 0. ],
#                           [0.26086957, 0.2173913 , 0.26086957, 0.        , 0.26086957, 0.        ],
#                           [0.        , 0.35294118, 0.29411765, 0.35294118, 0.        , 0.        ],
#                           [0.        , 0.        , 0.35294118, 0.29411765, 0.        , 0.35294118],
#                           [0.5       , 0.08333333, 0.        , 0.        , 0.41666667, 0.        ],
#                           [0.  , 0.        , 0.        , 0.16666667, 0.        , 0.83333333]]]])
#        calcd =  mkTransitions(env2, types2, 1, len(env2), gridworld_step_prob)
#        np.testing.assert_almost_equal(calcd, output, decimal=7, verbose=True)
#
#    def test_encode_decode(self):
#        N = randint(2,10)
#        X = randint(5,10)
#        states = np.random.choice(np.arange(X, dtype=np.uint64), N)
#        state_int = encodeJointState(states, X)
#        decoded = decodeJointState(state_int, N, X)
#        np.testing.assert_almost_equal(states, decoded, decimal=7, verbose=True)
#
#
