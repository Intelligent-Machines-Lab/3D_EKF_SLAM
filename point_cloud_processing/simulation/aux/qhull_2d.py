#!/usr/bin/python

# Compute the convex hull of a set of 2D points
# A Python implementation of the qhull algorithm
# 
# Tested with Python 2.6.5 on Ubuntu 10.04.4

# Copyright (c) 2008 Dave (www.literateprograms.org)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the "Software"), to deal 
# in the Software without restriction, including without limitation the rights 
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
# copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
# IN THE SOFTWARE.

from __future__ import division
import numpy as np

link = lambda a,b: np.concatenate((a,b[1:]))
edge = lambda a,b: np.concatenate(([a],[b]))

def qhull2D(sample, n =0):
    
    def dome(sample,base, n): 
        n = n+1
        #print('sample: ')
        #print(n)
        if n < 20:
            h, t = base
            dists = np.dot(sample-h, np.dot(((0,-1),(1,0)),(t-h)))
            outer = np.repeat(sample, dists>0, 0)
            if len(outer):
                pivot = sample[np.argmax(dists)]
                return link(dome(outer, edge(h, pivot), n), dome(outer, edge(pivot, t), n))
            else:
                return base
        else:
            return base

    if len(sample) > 2:
        axis = sample[:,0]
        base = np.take(sample, [np.argmin(axis), np.argmax(axis)], 0)
        return link(dome(sample, base, n), dome(sample, base[::-1], n))
    else:
        return sample

