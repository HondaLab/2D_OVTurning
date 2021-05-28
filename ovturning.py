#!/usr/bin/python3
# ovturning Yasushi Honda 2021 5/27
import math
import numpy as np

# Optimal Velocity model
class Optimal_Velocity_class:
    #インスタンス生成
    def __init__(self,parm):
        self.vs = parm[0]    #相互作用なしで動き続けるための項 
        self.a = parm[1]     #感応度
        self.alpha = parm[2] #最高速度決定
        self.beta = parm[3]  #αβで最適速度関数の変化率を決定
        self.b = parm[4]     #変曲点のx座標(ロボットの車頭距離にする)
        self.c = parm[5]     #前進後退の割合決定
        self.g = 1.0        # ロボットの半径(m)
        self.v = 0.1
        self.omega = 0.0
        print("#    a  alpha   beta      b      c      d")
        print("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f" % (self.a,self.alpha,self.beta,self.b,self.c,self.g))

    def calc(self,distance,theta,dt):

        f = self.alpha*(math.tanh(self.beta*(distance - self.b)) + self.c )
        ov = (1.0+math.cos(theta)) * f

        a = self.a*(self.vs + ov - self.v)

        self.v = self.v + dt*a
        self.omega = self.omega + dt*self.a*(theta-self.omega)

        left  = self.v + self.g *self.omega
        right = self.v - self.g *self.omega 

        # |left|<1, |right|<1 に規格化
        left = left/(2.0*self.alpha*(1+math.fabs(self.c)))
        right = right/(2.0*self.alpha*(1+math.fabs(self.c)))
        
        return left,right
