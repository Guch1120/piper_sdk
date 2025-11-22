#!/usr/bin/env python3
# -*-coding:utf8-*-

from enum import Enum, auto

class CanIDPiper(Enum):
    '''
    msg_v2
    
    ロボットアームCAN ID
    '''
    '''
    msg_v2
    
    piper's can_ids
    '''
    # アクティブフィードバックコマンド、全体オフセットを0x2B1~0x2B8または0x2C1~0x2C8に設定可能、詳細はコマンド0x470を参照
    ARM_STATUS_FEEDBACK = 0x2A1         #ロボットアームステータスフィードバックID
    ARM_END_POSE_FEEDBACK_1 = 0x2A2     #ロボットアームエンドポーズフィードバック
    ARM_END_POSE_FEEDBACK_2 = 0x2A3
    ARM_END_POSE_FEEDBACK_3 = 0x2A4
    ARM_JOINT_FEEDBACK_12 = 0x2A5       #ロボットアーム関節フィードバック
    ARM_JOINT_FEEDBACK_34 = 0x2A6
    ARM_JOINT_FEEDBACK_56 = 0x2A7
    ARM_GRIPPER_FEEDBACK = 0x2A8        #ロボットアームグリッパーフィードバック
    # 運動制御コマンド、全体オフセットを0x160~0x169または0x170~0x179に設定可能、詳細はコマンド0x470を参照
    ARM_MOTION_CTRL_1 = 0x150
    # ARM_STOP_CTRL = 0x150               #ロボットアーム緊急停止
    # ARM_TRACK_CTRL = 0x150              #ロボットアーム軌道コマンド
    # ARM_GRAG_TEACH_CTRL = 0x150         #ロボットアームドラッグティーチングコマンド
    ARM_MOTION_CTRL_2 = 0x151
    # ARM_MODE_CTRL = 0x151               #ロボットアーム制御モード
    # ARM_MOVE_MODE_CTRL = 0x151          #ロボットアームModeモード
    # ARM_MOVE_SPD_RATE_CTRL = 0x151      #ロボットアーム運動速度パーセンテージ
    ARM_MOTION_CTRL_CARTESIAN_1=0x152#ロボットアーム運動制御直交座標系コマンド1,X&Y
    ARM_MOTION_CTRL_CARTESIAN_2=0x153#ロボットアーム運動制御直交座標系コマンド1,Z&RX
    ARM_MOTION_CTRL_CARTESIAN_3=0x154#ロボットアーム運動制御直交座標系コマンド1,RY&RZ
    ARM_JOINT_CTRL_12=0x155             #ロボットアーム関節制御コマンド12,J1&J2
    ARM_JOINT_CTRL_34=0x156             #ロボットアーム関節制御コマンド12,J3&J4
    ARM_JOINT_CTRL_56=0x157             #ロボットアーム関節制御コマンド12,J5&J6
    ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL=0x158#円弧モード座標番号更新コマンドデータ
    ARM_GRIPPER_CTRL = 0x159            #グリッパー制御コマンド
    #----------------------------------V1.5-2バージョン以降---------------------------------------------#
    ARM_JOINT_MIT_CTRL_1 = 0x15A
    ARM_JOINT_MIT_CTRL_2 = 0x15B
    ARM_JOINT_MIT_CTRL_3 = 0x15C
    ARM_JOINT_MIT_CTRL_4 = 0x15D
    ARM_JOINT_MIT_CTRL_5 = 0x15E
    ARM_JOINT_MIT_CTRL_6 = 0x15F
    #---------------------------------------------------------------------------------------------#
    # ロボットアームパラメータ設定および設定コマンド
    # コマンド名にフィードバック、応答が含まれる場合; 意思決定制御ユニット->ロボットアームメインコントローラ
    # コマンド名にクエリ、設定が含まれる場合; ロボットアームメインコントローラ->意思決定制御ユニット
    ARM_MASTER_SLAVE_MODE_CONFIG = 0x470
    # ARM_MS_LINKAGE_CONFIG = 0x470          #追従マスタースレーブモード設定コマンド-連動設定コマンド
    # ARM_MS_FEEDBACK_INSTRUCTION_OFFSET_CONFIG = 0x470#追従マスタースレーブモード設定コマンド-フィードバックコマンドオフセット値
    # ARM_MS_CTRL_INSTRUCTION_OFFSET_CONFIG = 0x470#追従マスタースレーブモード設定コマンド-制御コマンドオフセット
    # ARM_MS_LINKAGE_CTRL_OFFSET_CONFIG = 0x470#追従マスタースレーブモード設定コマンド-連動モード制御目標アドレスオフセット値
    # ティーチング入力アームに設定後、アクティブ周期フィードバックメッセージIDにオフセットを追加（オフセット量は設定可能）、モードは連動ティーチング入力モードに切り替わり、制御コマンドに応答せず、関節モード制御コマンドをアクティブに送信します；
    # 運動出力アームに設定後、通常状態に戻ります（連動ティーチング入力モードを終了し、待機モードに入ります）；このコマンドを受信していないロボットアームはデフォルトでこの状態になります
    ARM_MOTOR_ENABLE_DISABLE_CONFIG = 0x471     #モーター有効化コマンド
    # ARM_MOTOR_DISABLE_CONFIG = 0x471    #モーター無効化コマンド
    # ARM_SEARCH_MOTOR_ANGLE_CONFIG = 0x472 #モーター角度クエリ
    ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT = 0x472 #モーター角度/最大速度/加速度制限クエリ
    # ARM_SEARCH_MOTOR_MAX_ACC_CONFIG = 0x472 #モーター最大加速度制限クエリ
    ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD = 0x473 #現在のモーター最大角度制限、最小角度制限、最大関節速度をフィードバック
    ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET = 0x474      #モーター角度制限/最大速度設定コマンド
    ARM_JOINT_CONFIG = 0x475            #関節設定コマンド
    ARM_INSTRUCTION_RESPONSE_CONFIG=0x476#設定コマンド応答
    ARM_FEEDBACK_RESP_SET_INSTRUCTION = 0x476 #設定コマンド応答フィードバック
    ARM_PARAM_ENQUIRY_AND_CONFIG = 0x477#ロボットアームパラメータクエリおよび設定コマンド
    ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM = 0x478    #現在の末端速度/加速度パラメータをフィードバック
    ARM_END_VEL_ACC_PARAM_CONFIG = 0x479      #末端速度/加速度パラメータ設定コマンド
    ARM_CRASH_PROTECTION_RATING_CONFIG=0x47A#衝突保護レベル設定コマンド
    ARM_CRASH_PROTECTION_RATING_FEEDBACK=0x47B#衝突保護レベルフィードバックコマンド
    ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT=0x47C#現在のモーター最大加速度制限をフィードバック
    #----------------------------------V1.5-2バージョン以降---------------------------------------------#
    ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG = 0x47D
    ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK = 0x47E
    #---------------------------------------------------------------------------------------------#
    ARM_FEEDBACK_JOINT_VEL_ACC_1 = 0x481         #現在の関節の末端速度/加速度をフィードバック
    ARM_FEEDBACK_JOINT_VEL_ACC_2 = 0x482
    ARM_FEEDBACK_JOINT_VEL_ACC_3 = 0x483
    ARM_FEEDBACK_JOINT_VEL_ACC_4 = 0x484
    ARM_FEEDBACK_JOINT_VEL_ACC_5 = 0x485
    ARM_FEEDBACK_JOINT_VEL_ACC_6 = 0x486
    #ライト制御0x1ノードID、フレームID 0x121
    ARM_LIGHT_CTRL = 0x121              #ライト制御コマンド
    #ドライバ情報高速フィードバック、ノードID 0x1~0x6
    ARM_INFO_HIGH_SPD_FEEDBACK_1 = 0x251
    ARM_INFO_HIGH_SPD_FEEDBACK_2 = 0x252
    ARM_INFO_HIGH_SPD_FEEDBACK_3 = 0x253
    ARM_INFO_HIGH_SPD_FEEDBACK_4 = 0x254
    ARM_INFO_HIGH_SPD_FEEDBACK_5 = 0x255
    ARM_INFO_HIGH_SPD_FEEDBACK_6 = 0x256
    #ドライバ情報低速フィードバック、ノードID 0x1~0x6
    ARM_INFO_LOW_SPD_FEEDBACK_1 = 0x261
    ARM_INFO_LOW_SPD_FEEDBACK_2 = 0x262
    ARM_INFO_LOW_SPD_FEEDBACK_3 = 0x263
    ARM_INFO_LOW_SPD_FEEDBACK_4 = 0x264
    ARM_INFO_LOW_SPD_FEEDBACK_5 = 0x265
    ARM_INFO_LOW_SPD_FEEDBACK_6 = 0x266
    #CANアップグレードバスサイレントモード設定コマンド
    ARM_CAN_UPDATE_SILENT_MODE_CONFIG=0x422
    # ファームウェア読み取りコマンド
    ARM_FIRMWARE_READ = 0x4AF
    def __str__(self):
        return f"{self.name} (0x{self.value:X})"
    def __repr__(self):
        return f"{self.name}: 0x{self.value:X}"