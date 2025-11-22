#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームの全モーターの最大加速度制限を読み取ります
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    while True:
        piper.SearchAllMotorMaxAccLimit()
        print(piper.GetAllMotorMaxAccLimit())
        time.sleep(0.01)
    