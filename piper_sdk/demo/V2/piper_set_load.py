#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# ロボットアームの負荷を設定します
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    load = 2    # 0，1，2
    piper.ArmParamEnquiryAndConfig(0, 0, 0, 0xAE, load) # 0xFC
    