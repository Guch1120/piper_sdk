#!/usr/bin/env python3
# -*-coding:utf8-*-
# このデモはシリアルCANメッセージを読み取ります
# 指定されたパラメータのCANバスを使用します
# 注意: interfaceのcan_auto_initがFalseの場合、ConnectPortを実行する前にCreateCanBusを実行して内部の__arm_canを初期化する必要があります。そうしないとエラーが発生します。
# PCIe-CANまたはシリアルCANモジュールを使用する場合、judge_flagをFalseに設定する必要があります。そうしないと、Linuxシステム下のsocketcanモジュールが検出されます。
# 注意: 先にシリアルポートの権限を付与する必要があります: sudo chmod 777 /dev/ttyACM0
# 通常のフレームレートは約3040です（シングルアーム接続時）
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2(can_auto_init=False)
    piper.CreateCanBus(can_name="/dev/ttyACM0",
                       bustype="slcan",
                       expected_bitrate=1000000,
                       judge_flag=False
                       )
    piper.ConnectPort(piper_init=False)
    while(True):
        print(f"all_fps: {piper.GetCanFps()}")
        time.sleep(0.01)
    