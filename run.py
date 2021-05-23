#! /usr/bin/python3
#  run.py CC BY-SA Yasushi Honda 2021 5/23
#  2dovr_210513.py
#  2021-04-16
#  Masashi Yamada

#  各モジュールインポート
import csv
import time
import math
import sys
import cv2
import numpy as np
#  Pythonファイルインポート 
import ovturning as OVT         # 2次元最適速度モデル関係
#import pixy_210416 as PIXY_py       # Pixyカメラ関係
import picam as PICAM_py # picamera関係
import modules.motor5a as mt         #  モーターを回転させるためのモジュール
#import modules.vl53_4a as lidar     #  赤外線レーザーレーダ 3つの場合
import modules.vl53_3a as lidar      #  赤外線レーザーレーダ 2つの場合

#sokcet tuusinn kannkei
import socket
#import socket1a as sk

print("# ２次元最適速度ロボット、走行プログラム")

select_hsv = "y"
show_period = 0.1
output_file="result.xy"

SLEEP = 0.2
EX_TIME = 3    #  (min)
BUS = 1         # bus number
I2C_ADDR = 0x54 #I2Cアドレス
GPIO_L = 17     #  左モーターのgpio 17番
GPIO_R = 18     #  右モーターのgpio 18番
MAX_SPEED = 50  # パーセント
DT = 0.03  # dtの初期値
dt = DT # dtは毎回観測する

#  パラメータ記載のファイルの絶対パス
FILE = "parm.csv" 


#  実験パラメータ読み込み
def Parameter_read(file_path):
    tmp = []
    reader = csv.reader(file_path)
    header = next(reader)
    for row in reader:
        if len(row) == 0:
           pass 
        else:
            tmp.append(float(row[0]))
            tmp.append(float(row[1]))
            tmp.append(float(row[2]))
            tmp.append(float(row[3]))
            tmp.append(float(row[4]))
            tmp.append(float(row[5]))
    return tmp
#  物体未認識時のhyperbolic-tan
def tanh(x):
    """
    alpha=30.0
    alpha2=30.0
    beta=0.004 #  0.004
    beta2=10.00
    b=160  #  280
    c=0
    f=alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c
    """
    delta = 0.1 #  beta
    p = 250     #  b
    q = 0.0     #  c
    f = (math.tanh(delta * (x - p) ) + q )
    return f

#  各変数定義
parm = []

#  パラメータ読み込み
file_pointer = open(FILE,'r')
parm = Parameter_read(file_pointer)

#  インスタンス生成
ovt = OVT.Optimal_Velocity_class(parm)         #  2次元最適速度モデル関係
#tofR,tofL,tofC=lidar.start() #  赤外線レーザ(3)
tofL,tofR=lidar.start()       #  赤外線レーザ(2)
print("VL53L0X 接続完了\n")
#time.sleep(2)
picam =PICAM_py.PI_CAMERA_CLASS() 
print("picamera 接続完了\n")

out=open(output_file,"w")

#time.sleep(2)
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

count = 0
data = []

# Color object の指定
if select_hsv=='y':
    lower_light,upper_light=picam.calc_hsv()
else:
    #Red Cup H:S:V=3:140:129
    # h,s,v = 171,106,138
    H = 174; S = 151; V = 172
    h_range = 10; s_range = 80; v_range = 60 # 明度の許容範囲
    lower_light = np.array([H-h_range, S-s_range, V-v_range])
    upper_light = np.array([H+h_range, S+s_range, V+v_range])

now = time.time()
start = now
init=now

vl=0; vr=0

print("停止するためには 'q' を押してください")
print("#  time",end="")
print("   rate",end="")
print(" dist(m) theta(rad)",end="")
print("  vL",end="")
print("      vR",end="")
print("    vL/vR")
string="# time"
string+="    dist"
string+="   theta"
string+="      vl"
string+="      vr \n"
out.write(string)
key=cv2.waitKey(1)
while key!=ord("q"):
    dist,theta,frame = picam.calc_dist_theta(lower_light, upper_light)
    count = count + 1
    if dist != None:
        mode = "picam"
        dist = float(dist)
        # pixyカメラで物体を認識している時
        vl, vr = ovt.calc(dist,theta,dt)

    else:
        pass
        """
        mode = "VL53L0X"
        #print(mode)
        lidar_distanceL=tofL.get_distance()
        if lidar_distanceL>2000:
            lidar_distanceL=2000
          
        lidar_distanceR=tofR.get_distance()
        if lidar_distanceR>2000:
            lidar_distanceR=2000

        vr = tanh(lidar_distanceL)
        vl = tanh(lidar_distanceR)
        print("\r %s v_L=%6.2f v_R=%6.2f" % (mode,vl,vr),end="")
        """

    last = now
    now = time.time()
    dt = now-last
    if now-init>show_period and dist!=None and theta!=None:
    #if vl>1 or vl<-1 or vr>1 or vr<-1 :
        init=now
        print("\r %6.3f %6d" % (now-start,int(1/dt)),end="")
        #print(" %s " % mode,end="")
        print(" %6.2f " % dist, end="")
        print(" %6.2f " % theta, end="")
        #print(" %8.4f " % d_theta, end="")
        print(" %6.2f " % vl, end="")
        print(" %6.2f " % vr, end="")
        print(" %7.3f " % (vl/vr),end="")

        string="{0:6.2f}, ".format(now-start)
        string+="{0:6.3f}, ".format(dist)
        string+="{0:6.3f}, ".format(theta)
        string+="{0:6.3f}, ".format(vl)
        string+="{0:6.3f}\n".format(vr)
        out.write(string)

    vl = vl * MAX_SPEED
    vr = vr * MAX_SPEED
    if vl > 100:  # 左モータに対する
        vl =100   # 閾値処理
    if vl < -100: # -1 < v_l < 1
        vl = -100 #
      
    if vr > 100:  # 右モータに対する
        vr =100   # 閾値処理
    if vr < -100: # -1 < v_r < 1
        vr = -100 #

    if dist!=None and theta!=None:
       mL.run(vl)
       mR.run(vr)


    cv2.imshow("frame",frame)
    key=cv2.waitKey(1)
    #time.sleep(DT)

mR.stop()
mL.stop()

out.close()

print("\n")
print("===============================")
print("=  実験時間 {:.1f} (sec)".format(now-start))
print("=  q_s--->")
print("===============================")
print()
print("おつかれさまでした  ^-^/")

#print("測定回数--->",count)
#print("測定レート {:.3f} (回数/sec)".format(count/15))
