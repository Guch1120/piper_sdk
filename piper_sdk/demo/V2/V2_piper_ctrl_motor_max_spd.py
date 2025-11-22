#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# 特定のモーターの最大速度を個別に設定します
# 注意: このコマンドはプロトコルを通じてドライバのフラッシュに直接書き込まれるため、リアルタイムで更新することはできません。速度を動的に調整する必要がある場合は、位置速度モードの速度パーセンテージを使用してください。
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    # 3rad/s
    for i in range(1,7):
        piper.MotorMaxSpdSet(i, 3000)
        time.sleep(0.1)
    while True:
        piper.SearchAllMotorMaxAngleSpd()
        print(piper.GetAllMotorAngleLimitMaxSpd())
        time.sleep(0.01)
    