# -*- coding: utf-8 -*-
"""
Created on Sun May 26 13:50:22 2019

@author: MK
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

def func(q,tau,avar):
    return 3.*q[0]/tau**2 + q[1]/tau + q[2]/3.*tau - avar
def adev(q,tau):
    return np.sqrt(3.*q[0]/tau[:]**2 + q[1]/tau[:] + q[2]/3.*tau[:])


file = open('adevfree.txt','r')
line = file.readline()
_tau = []
_avar = []
for line in file:
    a = line.split()
    if len(a) >= 2:
        _tau.append(float(a[0]))
        _avar.append(float(a[2])**2*1e26)
file.close()

tau = np.array(_tau)
avar = np.array(_avar)

q_init = np.array([1e-18, 1e-30, 1e-40])*1e26
opt = least_squares(func, q_init, args = (tau, avar), ftol = 1e-15)
q = opt.x*1e-26
q[0] = q[0]
q[1] = q[1]
print(q)
for i in range(len(q)):
    if q[i] < 0:
        q[i] = 0

plt.loglog(tau, 1e-13*np.sqrt(avar))
plt.loglog(tau, adev(q, tau))