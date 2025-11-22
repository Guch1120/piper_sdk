#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# 特定のモーターの関節リミットを個別に設定します
# 注意: このコマンドはプロトコルを通じてドライバのフラッシュに直接書き込まれるため、リアルタイムで更新することはできません
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    
    piper.MotorAngleLimitMaxSpdSet(1, 1500, -1500)
    piper.MotorAngleLimitMaxSpdSet(2, 1800, 0)
    piper.MotorAngleLimitMaxSpdSet(3, 0, -1700)
    piper.MotorAngleLimitMaxSpdSet(4, 1000, -1000)
    piper.MotorAngleLimitMaxSpdSet(5, 700, -700)
    piper.MotorAngleLimitMaxSpdSet(6, 1700, -1700)

    while True:
        print(piper.GetAllMotorAngleLimitMaxSpd())
        time.sleep(0.1)
    