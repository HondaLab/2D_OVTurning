このレポジトリは，2ホイール，または[スキッドステアロボット](https://github.com/HondaLab/SSR2)のための自律走行プログラムを開発するためのものです．

<img src='https://github.com/HondaLab/2D_OVTurning/blob/honda/SSR2.JPG' width=600>

前面に設置されたpicamera画像から色オブジェクトまでの相対距離dと相対角度thetaの値を検出します．
それらの値を基に，最適速度(Optimal Velocity)で左右のホイール速度 vL,vRを決めるアルゴリズムです．
(2021 5/23)


## Optimal Velocity Turning Algorithm (OVT)
このアルゴリズムでは，
速度を，速度の大きさ v と 旋回角速度 omegaで表すことにします．

<img src='https://github.com/HondaLab/2D_OVTurning/blob/honda/OVTurning.JPG' width=600>

ロボットの前面にあるPicameraによって，dとthetaを観測します．
[プログラム run.py](https://github.com/HondaLab/2D_OVTurning/blob/main/run.py)のなかで，

```
dist,theta,frame=picam.calc_dist_theta(xxx,yyy)
```
の部分がそれに当たります，下記ではdistのことをdと略記します．
  * f = alpha[tanh{beta(d-b)}+c]
  * V = (1+cos theta) f
  * v' = v + a(V-v)
  * omega' = omega + a(theta - omega)
  * vL = v + r g omega  
  * vR = v - r g omega
  * g: スリップを考慮したゲイン

vL,vRの値が左右のモーターに与える出力値．

## 構成
  * run.py: このプログラムを実行する
  * picam.py: picameraでd,thetaを求めるクラス記述
  * modules/: motor5a.py など
  * ovt.py: Optimal Velocity Turning のクラス記述
  * parm.csv: OV model パラメータ
  * stop.py: ロボットの緊急停止

## 実行結果例
<img src='https://github.com/HondaLab/2D_OVTurning/blob/honda/result.png' width=600>
  
