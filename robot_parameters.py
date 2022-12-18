import numpy as np
import pandas
import math

class Robot_Parameters():
    def __init__(self):
        self.alpha=[-math.pi/2,math.pi,math.pi,-math.pi/2,-math.pi/2,0]
        self.d=[0.37,0,0,0.55,0.37,0]
        self.a=[0,0.165,0.120,0,0,0]
        self.kp=0.03
        self.kd=1

    def get_a(self):
        return self.a
    
    def get_d(self):
        return self.d
    
    def get_alpha(self):
        return self.alpha
    
    def get_kpkd(self):
        return self.kp,self.kd

    

