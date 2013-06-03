# -*- coding: utf-8 -*-
"""
Created on Sun May 19 22:37:18 2013

@author: Andrew
"""
import numpy as np, serial as ps, matplotlib.pyplot as plt, math, time
from mpl_toolkits.mplot3d import Axes3D

class KFGyroAcc(object):
    
    def __init__(self):

#        self.ser = ps.Serial('/dev/tty.usbmodemfd121', 9600)
        self.ser = ps.Serial('/dev/tty.usbmodemfa131', 9600)
        
        # sampling interval        
        T = 0.1;
        
        # state transition matrix (constant acceleration model)
        self.F = np.array([[1.0,   T, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0,   T, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0,   T],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        
        # state transition noise covariance
        sigmaQ = 100
        G = np.array([[T*T / 2.0,       0.0,       0.0], 
                      [        T,       0.0,       0.0],
                      [      0.0, T*T / 2.0,       0.0],
                      [      0.0,         T,       0.0], 
                      [      0.0,       0.0, T*T / 2.0],
                      [      0.0,       0.0,         T]])
                      
        self.Q = sigmaQ * sigmaQ * np.dot(G, G.transpose())

        # observation matrix (last two rows change at each iteration)
        self.HG = np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        self.HGA = np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                             [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
          
        # observation noise 
        sigmaGyro = 0.15       
        self.RG = sigmaGyro**2 * np.identity(3)
        self.RGA = sigmaGyro * sigmaGyro * np.identity(5)
        sigmaAcc = 5
        self.RGA[3,3] = sigmaAcc * sigmaAcc
        self.RGA[4,4] = sigmaAcc * sigmaAcc
        
        # initial state
        self.xG = np.zeros(6)
        self.xGA = np.zeros(6)
        
        #initial state covariance
        self.PG = np.identity(6)
        self.PGA = np.identity(6)
        
        # non-filter estimate initial state
        self.xNoFilt = np.zeros(6)
        
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection = '3d')
        
        self.axTypes = [['r', 'g', 'b'], 
                        ['ro', 'go', 'bo'], 
                        ['r:', 'g:', 'b:']]
                        
        self.axis2 = np.array([2, -2])
        self.axis0 = np.array([0, 0])
           
        self.filtList = ['G', 'GA']
        self.plotList = ['G', 'NoFilt', 'GA']
        
    def getRot(self, strID):

        x = eval('self.x' + strID + '.copy()')
        
        u = math.pi / 180.0 * np.array([x[0], x[2], x[4]])  
        
        theta = np.linalg.norm(u)
        
        if theta > 1e-2:
            
            u = u / theta
            
            uOut = np.outer(u, u)
            
            uSkew = np.array([[0.0, -u[2], u[1]], 
                              [u[2], 0.0, -u[0]],
                              [-u[1], u[0], 0.0]])
        
            Rot = (math.cos(theta) * np.identity(3) + 
                math.sin(theta) * uSkew + 
                (1 - math.cos(theta)) * uOut)
                
        else:
            
            Rot = np.identity(3)
            
        return Rot
        
        
                
    def dispRot(self):

        self.ax.cla()
        
        self.ax.plot(3*self.axis2, self.axis0, self.axis0, self.axTypes[2][0])
        self.ax.plot(self.axis0, self.axis2, self.axis0, self.axTypes[2][1])
        self.ax.plot(self.axis0, self.axis0, self.axis2, self.axTypes[2][2])
        
#        R = self.getRot('G')
        
        for idx, filtType in enumerate(self.plotList):
            
            R = self.getRot(filtType)
            for ii in range(3):
            
                n0 = np.array([R[0, ii], -R[0, ii]]) + 4*idx - 4
                n1 = np.array([R[1, ii], -R[1, ii]])
                n2 = np.array([R[2, ii], -R[2, ii]])
                
                self.ax.plot(n0, n1, n2, self.axTypes[0][ii])                        
                self.ax.scatter(n0, n1, n2, self.axTypes[1][ii])

        plt.draw()
        
        
    def kf(self, z, strID):
        
        x = eval('self.x' + strID + '.copy()')
        P = eval('self.P' + strID + '.copy()')
        H = eval('self.H' + strID + '.copy()')
        R = eval('self.R' + strID + '.copy()')
        
        # propagating state and state covariance
        xp = np.dot(self.F, x)
        Pp = np.dot(np.dot(self.F, P), self.F.transpose()) + self.Q
        
        # calculating Jacobian of measurement equations and formatting the 
        # measurement
        if strID == 'GA':
            # Jacobian of H for extended part of filter 
            H[3,0] = 1250.0 * math.pi / 180.0 * math.cos(
                math.pi / 180.0 * x[0])
            H[4,2] = 1250.0 * math.pi / 180.0 * math.cos(
                math.pi / 180.0 * x[2])
            exec 'self.H' + strID + ' = H.copy()'
            
            meas = z.copy()
            
        else:
            
            meas = z[0:3].copy()
                
        # residual covariance
        S = np.dot(np.dot(H, Pp), H.transpose()) + R

        # Kalman gain
        K = np.dot(np.dot(Pp, H.transpose()),
                        np.linalg.inv(S))   
                                
        residual = meas - np.dot(H, xp)
        
        # updating state and state covariance        
        x = xp + np.dot(K, residual)
        P = Pp - np.dot(np.dot(K, H), Pp)
        
        # storing variables 
        exec 'self.xp' + strID + ' = xp.copy()'
        exec 'self.Pp' + strID + ' = Pp.copy()'
        exec 'self.S' + strID + ' = S.copy()'
        exec 'self.K' + strID + ' = K.copy()'
        exec 'self.x' + strID + ' = x.copy()'
        exec 'self.P' + strID + ' = P.copy()'        
        
    def measurementBasedUpdate(self, z):
        
        zx = z[3]
        zy = z[4]
        
        # accelerometer can only measure between -90 and 90 degrees, this keeps 
        # the measurement in that range (limits acceleration to 1g)
        zx = -1 if z[3] < -1250 else z[3] / 1250
        zx = 1 if zx > 1 else zx
        zy = -1 if z[4] < -1250 else z[4] / 1250
        zy = 1 if zy > 1 else zy  
             
        xAcc = 180.0 / math.pi * math.asin(zx)
        yAcc = 180.0 / math.pi * math.asin(zy)
        self.xNoFilt[1] = z[0]
        self.xNoFilt[3] = z[1]
        self.xNoFilt[5] = z[2]
        
        self.xNoFilt = np.dot(self.F, self.xNoFilt)
        
        # first order complimentary filter
        self.xNoFilt[0] = 0.98 * self.xNoFilt[0] + 0.02 * xAcc
        self.xNoFilt[2] = 0.98 * self.xNoFilt[2] + 0.02 * yAcc
        

    def getMeasurement(self):

        line = self.ser.readline().strip()

        z = np.array([float(x) for x in line.split(', ')])
                
        return z
        
    def runFilter(self):

        cnt = 1
        while True:
            
            z = self.getMeasurement()

            [self.kf(z, filtType) for filtType in self.filtList]
            
            self.measurementBasedUpdate(z)
            
            if (cnt % 5 == 0):
                
                self.dispRot()
                
            if (cnt % 10 == 0):
            
                ests = np.array([self.xNoFilt[0:6:2], self.xGA[0:6:2]])

                print ests
                cnt = 0
                
            cnt += 1
            
            
if __name__ == '__main__':

    temp = KFGyroAcc()
    
    temp.runFilter()

