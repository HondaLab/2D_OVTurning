このレポジトリは，2ホイール，またはスキッドステアロボットのための自律走行プログラムを開発するためのものです．
前面に設置されたpicamera画像から色オブジェクトまでの相対距離dと相対角度thetaの値を検出します．
それらの値を基に，左右のホイール速度 vL,vRを決めるアルゴリズムです．
(2021 5/23)

## Optimal Velocity Turning Algorithm
速度ベクトルを，速度の大きさ v と 旋回角速度 omegaで表す．
  * d,theta=picam.calc_dist_theta()
  * f = alpha[tanh{beta(d-b)}+c]
  * V = (1+cos theta) f
  * v' = v + a(V-v)
  * omega' = omega + a(theta - omega)
  * vL = v + r g omega  
  * vR = v - r g omega
  * g: スリップを考慮したゲイン
## 構成
  * run.py: このプログラムを実行する
  * picam.py: picameraでd,thetaを求めるクラス記述
  * modules/: motor5a.py など
  * ovt.py: Optimal Velocity Turning のクラス記述
  * parm.csv: OV model パラメータ
  * stop.py: ロボットの緊急停止
