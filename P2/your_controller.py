# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 3.32
        self.lf = 1.01
        self.Ca = 20000
        self.Iz = 29526.2
        self.m = 4500
        self.g = 9.81

        # Add additional member variables according to your need here.

    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g

        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        # ---------------|Lateral Controller|-------------------------
        """
        Please design your lateral controller below.
        """
        A = np.array([[0,1,0,0],[0,-4*Ca/(m*xdot),4*Ca/m,2*Ca*(lr-lf)/(m*xdot)],[0,0,0,1], [0,(2*Ca)*(lr-lf)/(Iz*xdot), (2*Ca)*(lf-lr)/Iz, (-2*Ca)*(lf**2 + lr**2)/(Iz*xdot)]])
        B = np.array([[0],[2*Ca/m],[0],[2*Ca*lf/Iz]])
        C = np.eye(4)
        D = np.zeros((4,1))
        
        desired_poles = np.array([-16,-4,-3,-2])

        K = signal.place_poles(A, B, desired_poles).gain_matrix
        
        # Find the closest node to the vehicle
        _, node = closestNode(X, Y, trajectory)
        
        # Choose a node that is ahead of our current node based on index
        nextNode = 176

        try:
            psiDesired = np.arctan2(trajectory[node+nextNode,1]-trajectory[node,1], trajectory[node+nextNode,0]-trajectory[node,0])

            e1 = (Y - trajectory[node+nextNode,1])*np.cos(psiDesired) - (X - trajectory[node+nextNode,0])*np.sin(psiDesired)
       
        except:
            psiDesired = np.arctan2(trajectory[-1,1]-trajectory[node,1], trajectory[-1,0]-trajectory[node,0])
           
            e1 = (Y - trajectory[-1,1])*np.cos(psiDesired) - (X - trajectory[-1,0])*np.sin(psiDesired)
            
        e1dot = ydot + xdot*wrapToPi(psi - psiDesired)
        e2 = wrapToPi(psi - psiDesired)
        e2dot = psidot 

        E = np.hstack((e1, e1dot, e2, e2dot)).reshape(4,1)
        
        # Calculate u = delta = -Kx
        delta = float(np.dot(-K,E))


        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
        """
        
        kp = 1200
        ki = 12
        kd = 10
        
        desiredXdot = 15
        xdotError = (desiredXdot - xdot)
        self.integralXdotError += xdotError
        derivativeXdotError = xdotError - self.previousXdotError
        self.previousXdotError = xdotError
        F = kp*xdotError + ki*self.integralXdotError*delT + kd*derivativeXdotError/delT


        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
