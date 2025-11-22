#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームをMIT制御モードに設定します。このモードでは、ロボットアームの応答が最も速くなります。
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    while True:
        piper.MotionCtrl_2(1, 1, 0, 0xAD)# 0xFC
        time.sleep(1)
    