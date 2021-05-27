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
の部分がそれに当たります．
下記に dist, theta から 左右のモーター出力 vL, vRを求めるアルゴリズムを示します．
ここではdistのことをdと略記します．
  * f = alpha[tanh{beta(d-b)}+c]
  * V = (1+cos theta) f
  * v' = v + a(V-v)
  * omega' = omega + a(theta - omega)
  * vL = v + r g omega  
  * vR = v - r g omega
   
   
gはホイールのスリップを考慮したゲインです．
f が最適速度の大きさです．最適速度の方向はthetaです．
alpha,beta,a,b,c,g は調整可能なパラメータです．

## パラメータの意味と典型的な値．
### alpha
最適速度関数の変化範囲を決めるパラメータです．
vの値は
```
-2(1+c)alpha < v < +2(1+c)alpha 
```
の範囲内で動きます．
この範囲を用いて後から，vL,vRは規格化されるので，alpha=1 として問題ありません．


### beta
最適速度関数の変曲点(d=b)における傾きが alpht*beta となります．
betaは，最適速度関数の滑らかさを決定します．
betaの値はロボットの動き方を大きく左右します．
経験的には概ね
```
0.1 < beta < 10
```
の範囲が妥当だと考えられます．

### a(感応度)
aがロボットの反応の強さを決めるゲインです．
この値もbetaと同じく，ロボットの動き方を大きく左右します．
経験的には
```
0.5 < a < 5
```
程度が妥当だと考えられます．
a=0にするとロボットは動きません．
またaが大きすぎると，ハンチングと呼ばれる振動現象を誘発します．

### b(期待車間距離)
ロボットとロボットの間隔が概ね b　となります．
d < b となるとロボットは速度を落とすか，停止，またはバックします．
ロボットのサイズに依存しますが， SSR2の場合
b = 0.3 (m) ぐらいが妥当だと考えられます．

### c
ロボット間の相互作用タイプを決めるパラメータです．
c>1とするとロボット間の相互作用が引力的になり，お互いに近づこうとします．

c<-1 の場合，逆に反発力的，すなわちロボットはお互いに離れようとします．

-1<c<1はその中間です．d<b では反発的，d>bでは引力的になります．

### g

## プログラムの構成
  * run.py: このプログラムを実行する
  * picam.py: picameraでd,thetaを求めるクラス記述
  * modules/: motor5a.py など
  * ovt.py: Optimal Velocity Turning のクラス記述
  * parm.csv: OV model パラメータ
  * stop.py: ロボットの緊急停止

## 実行結果例
<img src='https://github.com/HondaLab/2D_OVTurning/blob/honda/result.png' width=600>
  
