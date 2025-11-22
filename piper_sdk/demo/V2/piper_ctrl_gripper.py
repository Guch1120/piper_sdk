#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    piper.GripperCtrl(0,1000,0x03,0)
    time.sleep(0.1)
    range = 0
    count = 0
    while True:
        print(piper.GetArmGripperMsgs())
        count  = count + 1
        if(count == 0):
            print("1-----------")
            range = 0
        elif(count == 300):
            print("2-----------")
            range = 0.05 * 1000 * 1000 # 0.05m = 50mm
        elif(count == 600):
            print("3-----------")
            range = 0
            count = 0
        range = round(range)
        piper.GripperCtrl(abs(range), 1000, 0x03, 0)
        time.sleep(0.005)
    