#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームのリセットを設定します。MITモードまたはティーチングモードから位置速度制御モードに切り替えるときに実行する必要があります。
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    piper.MotionCtrl_1(0x02,0,0)#復帰
    