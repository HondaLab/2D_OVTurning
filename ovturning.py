#!/usr/bin/python3

# ovm_210522a CC BY-SA Yasushi Honda 2021 5/22
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
        self.d = 0.2        # ロボットの半径(m)
        self.vx = 0.0
        self.vy = 0.1
        print("#    a  alpha   beta      b      c      d")
        print("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f" % (self.a,self.alpha,self.beta,self.b,self.c,self.d))

    def calc(self,distance,theta,dt):

        f_rkj = self.alpha*(tanh(self.beta*(distance - self.b)) + self.c ) #(3)式

        nx = sin(theta)
        ny = cos(theta)

        ovx = (1+cos(theta)) * f_rkj * nx
        ovy = (1+cos(theta)) * f_rkj * ny

        ax = self.a * (self.vs + ovx - self.vx)
        ay = self.a * (self.vs + ovy - self.vy)

        vx_next = self.vx + dt * ax
        vy_next = self.vy + dt * ay

        v = sqrt_2(self.vx,self.vy) 
        v_next = sqrt_2(vx_next,vy_next)

        if v*v_next>0.020:# 速度ベクトルが小さすぎるときは d_theta=0
           out_z = vx_next * self.vy - vy_next * self.vx
           inner_v = self.vx * vx_next + self.vy * vy_next
           in_acos = (inner_v/(v*v_next))
           d_theta = sgn(out_z)*np.arccos(in_acos)
        else:
           d_theta=0.0

        d_theta=theta
        right = self.vy - (self.d * ( d_theta / dt) ) 
        left = self.vy + (self.d * ( d_theta / dt) )  

        self.vx = vx_next
        self.vy = vy_next
        
        # |left|<1, |right|<1 に規格化
        left = left/(2.0*self.alpha*(1+self.c))
        right = right/(2.0*self.alpha*(1+self.c))
        
        return left,right,d_theta,dt
