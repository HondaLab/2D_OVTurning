#!/usr/bin/python3
import modules.motor5a as mt

mL=mt.Lmotor(17) # 左のモーター
mR=mt.Rmotor(18) # 右のモーター

mL.move(0)
mR.move(0)
mL.stop()
mR.stop()
