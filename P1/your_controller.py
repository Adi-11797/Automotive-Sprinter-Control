# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
import math
from util import *

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 1.39
        self.lf = 1.55
        self.Ca = 20000
        self.Iz = 25854
        self.m = 1888.6
        self.g = 9.81
        
        # Add additional member variables according to your need here.

        self.integralPsiError = 0
        self.previousPsiError = 0   
        
        # Note: The next 2 variables are unused but already available in "base_controller.py"   
        
        self.integralXdotError = 0   
        self.previousXdotError = 0
        
        
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
        _, node = closestNode(X, Y, trajectory)

        nodeIndex = 95

        try:
            psiDesired = np.arctan2(trajectory[node+nodeIndex,1]-Y, \
            trajectory[node+nodeIndex,0]-X)
        except:
        # define an except condition for last node's global co-ordinates.
            psiDesired = np.arctan2(trajectory[-1,1]-Y, \
            trajectory[-1,0]-X)
            
        kp = 1.9
        ki = 0.001
        kd = 0.05
        
        psiError = wrapToPi(psiDesired-psi)
        self.integralPsiError += psiError
        derivativePsiError = psiError - self.previousPsiError
        delta = kp*psiError + ki*self.integralPsiError*delT + kd*derivativePsiError/delT
        delta = wrapToPi(delta)
        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
        """
        kp = 200
        ki = 30
        kd = 10
        
        desiredXdot = 9;
            
        xdotError = (desiredXdot - xdot)
        self.integralXdotError += xdotError
        derivativeXdotError = xdotError - self.previousXdotError
        self.previousXdotError = xdotError
        F = kp*xdotError + ki*self.integralXdotError*delT + kd*derivativeXdotError/delT


        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
