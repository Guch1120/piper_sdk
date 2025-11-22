#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
-------------------------------------------------
   File Name:    piper_set_joint_zero.py
   Description:  関節モーターのゼロ点位置を設定
   Author:       Jack
   Date:         2025-08-14
   Version:      1.0
   License:      MIT License
-------------------------------------------------
"""
import time
from piper_sdk import *

# テストコード
if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    time.sleep(0.1)
    print("設定プロセス中に'q'を入力するとプログラムを終了できます")
    print("During setup, enter 'q' to exit the program")
    print("ゼロ点を設定する前に指定されたモーターが無効化されます。ロボットアームを保護してください")
    print("Before setting zero position, the specified motor will be disabled. Please protect the robotic arm.")
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    piper.ModeCtrl(0x01, 0x01, 30, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    mode = -1
    while True:
        # モード選択
        if mode == -1:
            print("\nStep 1: 設定モードを選択してください(0: 指定モーター; 1: 順次設定): ")
            print("Step 1: Select setting mode (0: Single motor; 1: Sequential setting): ")
            mode = input("> ")
            if mode == '0':
                mode = 0
            elif mode == '1':
                mode = 1
            elif mode == 'q':
                break
            else:
                mode = -1
        
        # 単一モーター設定
        elif mode == 0:
            print("\nStep 2: ゼロ点を設定するモーター番号を入力してください(1~7)。7は全モーターを表します: ")
            print("Step 2: Enter motor number to set zero (1~7), 7 represents all motors: ")
            motor_num = input("> ")
            if motor_num == 'q':
                mode = -1
                continue
            try:
                motor_num = int(motor_num)
                if motor_num < 1 or motor_num > 7:
                    print("Tip: 入力が範囲外です")
                    print("Tip: Input out of range")
                    continue
            except:
                print("Tip: 整数を入力してください")
                print("Tip: Please enter an integer")
                continue
            piper.DisableArm(motor_num)
            print(f"\nInfo: 第{motor_num}号モーターの無効化に成功しました。モーターのゼロ点位置を手動で修正してください")
            print(f"Info: Motor {motor_num} disabled successfully. Please manually adjust to zero position")
            
            print(f"\nStep 3: Enterキーを押して第{motor_num}号モーターのゼロ点を設定します: ")
            print(f"Step 3: Press Enter to set zero for motor {motor_num}: ")
            if input("(Enterキーを押して続行/Press Enter) ") == 'q':
                mode = -1
                continue
            piper.JointConfig(motor_num, 0xAE)
            piper.EnableArm(motor_num)
            print(f"\nInfo: 第{motor_num}号モーターのゼロ点設定に成功しました")
            print(f"Info: Motor {motor_num} zero position set successfully")
        
        # 順次設定
        elif mode == 1:
            print("\nStep 2: 設定を開始するモーター番号を入力してください(1~6): ")
            print("Step 2: Enter starting motor number (1~6): ")
            motor_num = input("> ")
            if motor_num == 'q':
                mode = -1
                continue
            try:
                motor_num = int(motor_num)
                if motor_num < 1 or motor_num > 6:
                    print("Tip: 入力が範囲外です")
                    print("Tip: Input out of range")
                    continue
            except:
                print("Tip: 整数を入力してください")
                print("Tip: Please enter an integer")
                continue
            for i in range(motor_num, 7):
                piper.DisableArm(i)
                print(f"\nInfo: 第{i}号モーターの無効化に成功しました。モーターのゼロ点位置を手動で修正してください")
                print(f"Info: Motor {i} disabled successfully. Please manually adjust to zero position")
                
                print(f"\nStep 3: Enterキーを押して第{i}号モーターのゼロ点を設定します: ")
                print(f"Step 3: Press Enter to set zero for motor {i}: ")
                if input("(Enterキーを押して続行/Press Enter) ") == 'q':
                    mode = -1
                    break
                piper.JointConfig(i, 0xAE)
                piper.EnableArm(i)
                print(f"\nInfo: 第{i}号モーターのゼロ点設定に成功しました")
                print(f"Info: Motor {i} zero position set successfully")