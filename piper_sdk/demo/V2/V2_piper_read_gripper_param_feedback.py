#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
# V2バージョンSDK
# グリッパー/ティーチングペンダントパラメータ設定コマンド
# 初めてグリッパーまたはティーチングペンダントを使用するときは、これら2つのエンドエフェクタパラメータを設定する必要があります。そうしないと、データフィードバックがなくなり、アクチュエータを制御できなくなります。
# 通常、GripperTeachingPendantParamConfig関数の2番目のパラメータmax_range_configは70です。
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while True:
        piper.ArmParamEnquiryAndConfig(4)
        print(piper.GetGripperTeachingPendantParamFeedback())
        time.sleep(0.05)
    