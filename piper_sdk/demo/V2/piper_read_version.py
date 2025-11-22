#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意: デモは直接実行できません。実行するにはまずpipでSDKをインストールする必要があります。
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort(piper_init=True, start_thread=True)
    time.sleep(0.1) # ファームウェアのフィードバックフレームを読み取るのに時間が必要です。そうしないと、-0x4AFが返されます。ファームウェアバージョンを読み取る必要がない場合は、遅延を設定する必要はありません。
    print(f'=====>> Piper Current Interface Version is {piper.GetCurrentInterfaceVersion()} <<=====')
    print(f'=====>> Piper Current Interface Version is {piper.GetCurrentInterfaceVersion().value} <<=====')
    print(f'=====>> Piper Current Protocol Version is {piper.GetCurrentProtocolVersion()} <<=====')
    print(f'=====>> Piper Current Protocol Version is {piper.GetCurrentProtocolVersion().value} <<=====')
    print(f'=====>> Piper Current SDK Version is {piper.GetCurrentSDKVersion()} <<=====')
    print(f'=====>> Piper Current SDK Version is {piper.GetCurrentSDKVersion().value} <<=====')
    # ConnectPort(piper_init=True, start_thread=True) の場合のみ、ファームウェアバージョンを読み取ることができます。
    print(f'=====>> Piper Current Firmware Version is {piper.GetPiperFirmwareVersion()} <<=====')
    