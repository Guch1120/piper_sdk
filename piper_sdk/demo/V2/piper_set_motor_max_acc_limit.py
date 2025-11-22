#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# 特定のモーターの最大加速度を個別に設定します
# 注意: このコマンドはプロトコルを通じてドライバのフラッシュに直接書き込まれるため、リアルタイムで更新することはできません
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    # 5rad/s
    for i in range(1,7):
        piper.JointMaxAccConfig(i, 500)
        print(i)
        time.sleep(0.5) # データの書き込みには時間がかかります。前のフレームの設定コマンドを送信した後、しばらく遅延させる必要があります
    while True:
        piper.SearchAllMotorMaxAccLimit()
        print(piper.GetAllMotorMaxAccLimit())
        time.sleep(0.1)
    