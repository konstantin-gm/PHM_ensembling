#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  7 17:02:08 2021

@author: konstantin
"""

import numpy
from scipy import linalg
import numba
import pylab

def covar_by_noise(q, dt):
     Q = numpy.array([[q[1]**2*dt+q[2]**2*dt**3/3, q[2]**2*dt**2/2],
                      [q[2]**2*dt**2/2, q[2]**2*dt]], dtype=numpy.double)
     return Q
class Model:
    def __init__(self, noise1, noise2, control, dt, ctrl_interval, delay, d1, d2):
        self.noise1 = noise1
        self.noise2 = noise2
        self.Q1 = covar_by_noise(noise1, dt)
        self.L1 = linalg.cholesky(self.Q1)
        self.Q2  = covar_by_noise(noise2, dt)
        self.L2 = linalg.cholesky(self.Q2)
        self.G = control
        self.dt = dt
        self.ctrl_interval = ctrl_interval
        self.drift1 = d1
        self.drift2 = d2
        self.delay = delay
    #@numba.jit
    def calculate(self, N):        
        self.z = numpy.zeros((N, 1))
        self.z2 = numpy.zeros((N, 1))
        self.x1i = numpy.zeros((N, 1))
        self.x2i = numpy.zeros((N, 1))
        self.free1 = numpy.zeros((N, 1))
        self.free2 = numpy.zeros((N, 1))        
        
        F = numpy.array([[1, self.dt], [0, 1]])
        B = numpy.array([self.dt, 1])
        H = numpy.array([1, 0])
        D = numpy.array([0.5*self.dt**2, self.dt])
        
        # R = self.ref_noise[0]**2 + self.free_noise[0]**2
        # P = numpy.array([[R, 0], [0, self.free_noise[1]**2]])
        
        u1 = 0
        uprev1 = 0
        u2 = 0
        uprev2 = 0
        X1 = numpy.zeros(2)
        X2 = numpy.zeros(2)
        Xfree1 = numpy.zeros(2)
        Xfree2 = numpy.zeros(2)
        dX = numpy.zeros(2)
        
        zk = []
        uk = []
        symm = 0
        
        for i in range(N):
            free_w1 = numpy.random.randn(2)
            free_w2 = numpy.random.randn(2)
            du1 = 0*1e-17*numpy.random.randn(1)
            du2 = 0*1e-17*numpy.random.randn(1)
            pd1 = 1*1e-14*numpy.random.randn(1)
            pd2 = 1*1e-14*numpy.random.randn(1)
            wpn_free1 = numpy.random.randn(1)*self.noise1[0]
            wpn_free2 = numpy.random.randn(1)*self.noise2[0]
            self.z[i] = X1[0] - X2[0] + 1*wpn_free1 + wpn_free2 + pd1
            self.z2[i] = X1[0] - X2[0] + 1*wpn_free1 + wpn_free2 + pd1
            
            if i%self.ctrl_interval==0 and i >= 2*self.ctrl_interval+self.delay:
                u1 = self.G[0]*(self.z[i-self.delay*symm] - self.z[i-self.ctrl_interval-self.delay*symm])
                u1 += self.G[1]*(self.z[i-self.delay*symm] - 2*self.z[i-self.ctrl_interval-self.delay*symm] + self.z[i-2*self.ctrl_interval-self.delay*symm])
                u1 /= self.dt*self.ctrl_interval
                u1 = 1e-15*numpy.round(u1*1e15)
                u2 = self.G[0]*(self.z2[i-self.delay] - self.z2[i-self.ctrl_interval-self.delay])
                u2 += self.G[1]*(self.z2[i-self.delay] - 2*self.z2[i-self.ctrl_interval-self.delay] + self.z2[i-2*self.ctrl_interval-self.delay])
                u2 /= self.dt*self.ctrl_interval
                u2 = 1e-15*numpy.round(u2*1e15)
                uk.append(u1)
                zk.append(self.z[i])   
            else:
                u1 = 0
                u2 = 0
            
            X1 = F@X1 + self.L1@free_w1 - B*(u1+du1) + D*self.drift1
            X2 = F@X2 + self.L2@free_w2 + B*(u2+du2) + D*self.drift2
            Xfree1 = F@Xfree1 + self.L1@free_w1 + D*self.drift1
            Xfree2 = F@Xfree2 + self.L2@free_w2 + D*self.drift2
            
            uprev1 = u1
            uprev2 = u2
            
            self.x1i[i] = X1[0] + wpn_free1            
            self.x2i[i] = X2[0] + wpn_free2
            self.free1[i] = Xfree1[0] + wpn_free1
            self.free2[i] = Xfree2[0] + wpn_free2
                
        return self.x1i, self.x2i, self.free1, self.free2, numpy.array(uk), numpy.array(zk)



#@numba.jit('Tuple((float64[:], float64[:]))(float64[:], float64, float64[:])', cache=True)
def allan_deviation(z, dt, tau):
    ADEV = numpy.zeros(tau.size, dtype='double')
    n = z.size
    maxi = 0
    for i in range(tau.size):
        if tau[i]*3 < n:
            maxi = i
            sigma2 = numpy.sum((z[2*tau[i]::1] - 2*z[tau[i]:-tau[i]:1] + z[0:-2*tau[i]:1])**2)
            ADEV[i] = numpy.sqrt(0.5*sigma2/(n-2*tau[i]))/tau[i]/dt
        else:
            break
    return tau[:maxi].astype(numpy.double)*dt, ADEV[:maxi]


do_modeling = 1
adev = 1
save = 1
if (do_modeling):    
    q1 = numpy.sqrt(8.8e-26)
    q2 = numpy.sqrt(6.2e-36)
    free_noise = (numpy.sqrt(1e-25), q1, q2)
    
    dt = 1
    G = numpy.array([0.05, 0.01])
    print('[gx, gy] = ', G)
    ctrl_interval = 10
    model = Model(free_noise, free_noise, G, dt, ctrl_interval, 1, 5e-16/86400, 5e-16/86400)
    x1, x2, xfree1, xfree2, u, z = model.calculate(1000000)
    pylab.figure(1)
    pylab.plot(x1)
    pylab.plot(x2)
    # pylab.ylabel("Разность фаз, с")
    # pylab.xlabel("время")
    pylab.ylabel("Phase offset, s")    
    pylab.xlabel("time, s")
    pylab.figure(2)
    pylab.plot(u, 'r')
    # pylab.legend(("Разность частот (Калман)", "Управление"))
    # pylab.legend(("Frequency offset (Kalman)", "Control")) 
    pylab.xlabel("time, s")
    pylab.show()    
    if (adev):
        tau = numpy.arange(1,10)
        tau = numpy.append(tau, numpy.arange(10,100,10))
        tau = numpy.append(tau, numpy.arange(100,1000,100))
        tau = numpy.append(tau, numpy.arange(1000,10000,1000))
        tau = numpy.append(tau, numpy.arange(10000,100000,10000))
        tau = numpy.append(tau, numpy.arange(100000,1000000,100000))
        tau = numpy.append(tau, numpy.arange(1000000,10000000,1000000))
        taus, adev1 = allan_deviation(x1[100000:-1,0], dt, tau)
        taus, adev2 = allan_deviation(x2[100000:-1,0], dt, tau)
        taus, adev3 = allan_deviation(0.5*(x1[100000:-1,0]+x2[100000:-1,0]), dt, tau)
        taus, adev_free1 = allan_deviation(xfree1[100000:-1,0], dt, tau)
        taus, adev_free2 = allan_deviation(xfree2[100000:-1,0], dt, tau)        
        pylab.figure(3)
        pylab.loglog(taus, adev1, 'r')
        pylab.loglog(taus, adev2, 'g')
        #pylab.loglog(taus, adev3, 'b')
        pylab.loglog(taus, adev_free1, '--')
        pylab.loglog(taus, adev_free2, '--')
        # pylab.xlabel("интервал времени измерения, с")
        pylab.xlabel("averaging time, s")
        pylab.ylabel("ADEV")   
    pylab.figure(4)
    start = 100000
    dec = 10000
    pylab.plot((x1[start+dec:-1:dec] - x1[start:-dec:dec])/dec)
    pylab.plot((x2[start+dec:-1:dec] - x2[start:-dec:dec])/dec)
    pylab.plot((xfree1[start+dec:-1:dec] - xfree1[start:-dec:dec])/dec)
    pylab.plot((xfree2[start+dec:-1:dec] - xfree2[start:-dec:dec])/dec)
    
    yu = []
    sumu = 0
    for ui in u:
        sumu += ui[0]
        yu.append(sumu)
    yu = numpy.array(yu)
    xu = []
    sumu = 0
    for y in yu:
        sumu += y*ctrl_interval
        xu.append(sumu)
    xu = numpy.array(xu)
    #taus, adevu = allan_deviation(xu[1000:-1:1], 10, tau)
    #pylab.figure(3)
    #pylab.loglog(taus, adevu)
    #pylab.figure(5)
    #pylab.plot(yu)
    if save:        
        f = []
        f.append('tau.txt')
        f.append('adev1.txt')
        f.append('adev2.txt')
        f.append('adev_sum.txt')
        f.append('adev_free1.txt')
        f.append('adev_free2.txt')
        f.append('adev_u.txt')
        d = [taus, adev1, adev2, adev3, adev_free1, adev_free2]#, adevu]
        for i in range(6):
            f[i] = open(f[i],'w')
            for dat in d[i]:
                f[i].write(str(dat) + '\n')
            f[i].close()
  