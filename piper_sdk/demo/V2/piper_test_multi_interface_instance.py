#!/usr/bin/env python3
# -*-coding:utf8-*-
# 複数のロボットアームがある場合、複数のインスタンスを作成できます。can_portを識別することで、同じCANポートを読み取るインスタンスの作成を回避します。
# 注意: 複数のCANモジュールが必要です
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2("can_0")
    piper.ConnectPort(True)
    piper1 = C_PiperInterface_V2("can_1")
    piper1.ConnectPort(True)
    while True:
        print(piper.GetCanFps())
        time.sleep(1)
        pass
    