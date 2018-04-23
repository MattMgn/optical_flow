#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Name:        display_graph.py
# Purpose:     display pose and velocity from txt file with 4 columns [vx vy vz wpsi]
#
# Author:      Matthieu MAGNON
#
# Created:     April 2018
#
# Env : Python 2.7, 
#-------------------------------------------------------------------------------
# List of Classes:
#  displayGraph
#
# List of methodes:

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class Velocity:
    def __init__(self,vx,vy,vz,w):
        self.vx = vx;
        self.vy = vy;
        self.vz = vz;
        self.w  = w;

class Pose:
    def __init__(self,x,y,z,psi):
        self.x = x;
        self.y = y;
        self.z = z;
        self.psi  = psi;


class displayGraph:

    def __init__(self,file,dt):

        self.f = open(file,"r")
        self.dt = dt 

        self.extractVelocityBody()

        # display Velocity in Body frame
        self.n_plot = 1
        self.displayVelocity(self.vel_body, "Vitesses lineaires dans le repere mobile : Body")

        # compute Velocity in Earth frame
        self.vel_init = Velocity(0.0,0.0,0.0,0.0)
        self.computeVelocityEarth()

        # display Velocity in Earth frame
        self.displayVelocity(self.vel_earth, "Vitesses lineaires dans le repere fixe : Earth")

        # compute Pose
        self.pose_init = Pose(0.0,0.0,1.0,0.0)
        self.computePoseEarth()
        
        # display Pose in Earth frame
        self.displayPose()

        plt.show()
        

    def extractVelocityBody(self):
        vx = []
        vy = []
        vz = []
        w = []

        # extract columns
        for line in self.f:
            row = line.split()
            c = 1
            for col in row:
                if c==1:
                    vx.append( col )
                elif c==3:
                    vy.append( col )
                elif c==5:
                    vz.append( col )
                elif c==7:
                    w.append( col )
                c=c+1

        # convert as numpy arry
        vx = np.asarray(vx, dtype=float)
        vy = np.asarray(vy, dtype=float)
        vz = np.asarray(vz, dtype=float)
        w = np.asarray(w, dtype=float)

        # instance Velocity as vel
        self.vel_body = Velocity(vx,vy,vz,w)


    def displayVelocity(self,vel,title):
        # repere mobile Body
        plt.figure( self.n_plot )
        self.n_plot = self.n_plot + 1
        plt.subplot(211)
        plt.plot(vel.vx, label='vx_B')
        plt.plot(vel.vy, label='vy_B')
        plt.plot(vel.vz, label='vz_B')
        plt.title(title)
        plt.legend()

        plt.subplot(212)
        plt.plot(vel.w, label='w')
        plt.title("Vitesse angulaire")
        plt.legend()

        #plt.show()

    def computeVelocityEarth(self):

        # psi = delta_t * w
        psi = np.zeros(self.vel_body.vx.size)
        for k, w in zip( range(2,self.vel_body.vx.size), self.vel_body.w ):
            psi[k] = psi[k-1] + self.dt * w
        psi = 2*1.57*10.0*psi # coeff de calibration pix->deg

        # vx = cos(psi)*vx_B + sin(psi)*vy_B
        vx = np.multiply( np.cos(psi) , self.vel_body.vx ) + np.multiply( np.sin(psi) , self.vel_body.vy )
        # vy = -sin(psi)*vx_B + cos(psi)*vy_B
        vy = np.multiply( -np.sin(psi) , self.vel_body.vx ) + np.multiply( np.cos(psi) , self.vel_body.vy )

        self.vel_earth = Velocity(vx, vy, self.vel_body.vz, self.vel_body.w)


        
    def computePoseEarth(self):
        # in Earth coordinates
        x = np.zeros(self.vel_body.vx.size)
        y = np.zeros(self.vel_body.vx.size)
        z = np.zeros(self.vel_body.vx.size)
        psi = np.zeros(self.vel_body.vx.size)

        #init 
        x[1] = 0.0;
        y[1] = 0.0;
        z[1] = 1.0;
        psi[1] = 0.0;

        for k, vx, vy ,vz, w in zip(range(2,self.vel_earth.vx.size), self.vel_earth.vx, self.vel_earth.vy, self.vel_earth.vz, self.vel_earth.w):
            x[k] = x[k-1] + self.dt * vx
            y[k] = y[k-1] + self.dt * vy
            z[k] = z[k-1] + self.dt * vz
            psi[k] = psi[k-1] + self.dt * w


        self.pose = Pose(x,y,z, 2*1.57*10.0*psi)



    def displayPose(self):
        # repere fixe Earth
        plt.figure( self.n_plot )
        self.n_plot = self.n_plot + 1
        plt.subplot(211)
        plt.plot(self.pose.x, label='x_E')
        plt.plot(self.pose.y, label='y_E')
        plt.plot(self.pose.z, label='z_B')
        plt.title("Positions dans le repere fixe : Earth")
        plt.legend()

        plt.subplot(212)
        plt.plot(self.pose.psi, label='psi')
        plt.title("Orientation psi")
        plt.legend()

        #plt.show()

        # plot in X-Y
        plt.figure( self.n_plot )
        self.n_plot = self.n_plot + 1
        plt.plot(self.pose.x, self.pose.y, 'ro')
        plt.axis('equal') 
        plt.grid()
        plt.title("Positions dans le repere fixe : Earth")
        #plt.show()




if __name__ == '__main__':        

    displayGraph("/home/matt/rosbag/camera_pose_estimate.txt", 0.033)

    