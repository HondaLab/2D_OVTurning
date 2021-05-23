# Optimal Velocity Turning Algorithm
  2021 5/23
  速度ベクトルを，速度の大きさ v と 旋回角速度 omegaで表す．
    * d,theta=picam.calc_dist_theta()
    * f = alpha[tanh{beta(d-b)}+c]
    * V = (1+cos theta) f
    * v' = v + a(V-v)
    * omega' = omega + a(theta - omega)
    * vL = v + r g omega  
    * vR = v - r g omega
    * g: スリップを考慮したゲイン
