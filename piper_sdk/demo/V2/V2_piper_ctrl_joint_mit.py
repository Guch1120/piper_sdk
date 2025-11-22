#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# 特定のモーターのMIT制御を個別に設定します
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    
    while True:
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,-0.5,0,10,0.8,0)
        print(1)
        time.sleep(1)
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,0,0,10,0.8,0)
        print(2)
        time.sleep(1)
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,0.5,0,10,0.8,0)
        print(3)
        time.sleep(1)
    