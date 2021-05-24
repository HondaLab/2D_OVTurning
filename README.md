このレポジトリは，2ホイール，またはスキッドステアロボットのための自律走行プログラムを開発するためのものです．

<img src='https://github.com/HondaLab/2D_OVTurning/blob/honda/SSR2.JPG' width=600>

前面に設置されたpicamera画像から色オブジェクトまでの相対距離dと相対角度thetaの値を検出します．
それらの値を基に，最適速度(Optimal Velocity)で左右のホイール速度 vL,vRを決めるアルゴリズムです．
(2021 5/23)


## Optimal Velocity Turning Algorithm (OVT)
速度ベクトルを，速度の大きさ v と 旋回角速度 omegaで表す．

<img src='https://github.com/HondaLab/2D_OVTurning/blob/honda/OVTurning.JPG' width=600>

Picameraによる観測値は，dist, theta．
  * d,theta=picam.calc_dist_theta()
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
  
## ライセンス
[CC BY-SA 3.0](https://creativecommons.org/licenses/by-sa/3.0/deed.ja)
