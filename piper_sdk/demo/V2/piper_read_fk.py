#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2(dh_is_offset=1)
    piper.ConnectPort()
    # 使用前に有効化する必要があります
    piper.EnableFkCal()
    # 注意: 計算は単一スレッドで非常にリソースを消費するため、有効にするとCPU使用率がほぼ倍増します
    while True:
        # 1〜6番目の関節のポーズを表す6つの浮動小数点数のリストをフィードバックします。-1はjoint6のポーズを表します
        print(f"feedback:{piper.GetFK('feedback')[-1]}")
        print(f"control:{piper.GetFK('control')}")
        time.sleep(0.01)
    