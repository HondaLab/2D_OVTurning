#!/usr/bin/python3

# ovturning CC BY-SA Yasushi Honda 2021 5/22
"""
作成日：2020/10/31
更新日：2021/04/16
作成者：20043060 山田 将司
###説明###
ssr系ロボット用2次元最適速度モデル
"""
#モジュールインポート
import math
import numpy as np

# 定数定義
MAX = 3 #自身を除いたロボット最大数

# 平方根計算，math.と書きたくないため，自作
def sqrt_1(x):
    return math.sqrt(x)
def sqrt_2(x,y):
    return math.sqrt(x*x + y*y)

# 絶対値計算、math.と書きたくないため，自作
def fabs(x):
    return math.fabs(x)

# sin計算，math.と書きたくないため，自作
def sin(x):
    return math.sin(x)

# cos計算，math.と書きたくないため，自作
def cos(x):
    return math.cos(x)

# arc-cos計算，math.と書きたくないため，自作
def acos(x):
    try:
        return math.acos(x)
    except:
        return 0.0

# hyperbolic-tan計算，math.と書きたくないため，自作
def tanh(x):
    return math.tanh(x)

# 外積計算
def outer_product(a1,a2,b1,b2):
    outer_z = (a1*b2 - a2*b1) #OK
    return outer_z

# シグナム関数(符号関数)
def sgn(x):
    if x > 0:
        sign = 1
    if x == 0:
        sign = 0
    if x < 0:
        sign = -1
    return sign

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
        self.d = 1.0        # ロボットの半径(m)
        self.v = 0.1
        self.omega = 0.0
        print("#    a  alpha   beta      b      c      d")
        print("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f" % (self.a,self.alpha,self.beta,self.b,self.c,self.d))

    def calc(self,distance,theta,dt):

        f = self.alpha*(tanh(self.beta*(distance - self.b)) + self.c )
        ov = (1.0+cos(theta)) * f

        a = self.a*(self.vs + ov - self.v)

        self.v = self.v + dt*a
        self.omega = self.omega + dt*self.a*(theta-self.omega)

        left  = self.v + self.d *self.omega
        right = self.v - self.d *self.omega 

        # |left|<1, |right|<1 に規格化
        left = left/(2.0*self.alpha*(1+self.c))
        right = right/(2.0*self.alpha*(1+self.c))
        
        return left,right
