#!/usr/bin/env python3
# -*-coding:utf8-*-
#ロボットアームプロトコル解析基底クラス

from abc import ABC, abstractmethod
from enum import Enum, auto
import ctypes
import struct
from typing_extensions import (
    Literal,
)
from ..utils import *
from ..utils import logger, global_area

class C_PiperParserBase(ABC):
    '''
    Piperロボットアームデータ解析基底クラス
    '''
    '''
    Piper Robot Arm Data Parsing Base Class
    '''
    class ProtocolVersion(Enum):
        '''
        プロトコルバージョン列挙、派生クラスで指定する必要があります
        '''
        '''
        Protocol Version Enumeration, needs to be specified in the derived class.
        '''
        ARM_PROROCOL_V1 = auto()
        ARM_PROROCOL_V2 = auto()
        ARM_PROTOCOL_UNKNOWN = auto()
        def __str__(self):
            return f"{self.name} (0x{self.value:X})"
        def __repr__(self):
            return f"{self.name}: 0x{self.value:X}"
    
    def __init__(self) -> None:
        super().__init__()
        self.logger = LogManager.get_logger(global_area, f"ParserBase<{id(self)}>")
    
    @abstractmethod
    def DecodeMessage(self):
        '''
        メッセージをデコードし、CANデータフレームを設定されたタイプに変換します
        '''
        '''
        Decode the message, converting the CAN data frame into the specified type.
        '''
        pass
    
    @abstractmethod
    def EncodeMessage(self):
        '''
        メッセージをCANデータフレームに変換します
        
        入力データをCANデータのIDとデータに変換するだけで、CANメッセージのchannel、dlc、is_extended_idには値を割り当てません
        '''
        '''
        Convert the message into a CAN data frame.

        This function only converts the input data into the id and data of the CAN message, without assigning values to channel, dlc, or is_extended_id for the CAN message.
        '''
        pass

    @abstractmethod
    def GetParserProtocolVersion(self):
        '''
        現在のプロトコルバージョンを取得します
        '''
        '''
        Get the current protocol version.
        '''
        pass
    
    def ConvertToNegative_8bit(self, value: int, signed:bool=True) -> int:
        '''
        入力された整数を8ビット整数に変換します。
        入力 value 範囲:[0,255]
        signed が True の場合、8ビット符号付き整数に変換されます。[-128, 127]
        signed が False の場合、8ビット符号なし整数に変換されます。[0, 255]
        '''
        '''
        Convert the input integer to an 8-bit integer.
        input value range:[0,255]
        If signed is True, it will be converted to a 32-bit signed integer. [-128, 127]
        If signed is False, it will be converted to a 32-bit unsigned integer. [0, 255]
        '''
        # 範囲チェック
        if not (0 <= value <= 255):
            self.logger.error("Error ConvertToNegative_8bit:  Input value exceeds the range [0, 255].")
        # 8ビット符号なし整数に変換
        value &= 0xFF  # value を 8 ビット符号なし整数に変換
        if signed:
            if value & 0x80:  # 符号ビットをチェック
                value -= 0x100  # 符号ビットが 1 の場合、負数を表すため、2^8 を減算する必要があります
        return value

    def ConvertToNegative_int8_t(self, value: int) -> int:
        '''
        入力された整数を8ビット符号付き整数に変換します。
        入力 value 範囲:[0,255]
        return範囲: [-128, 127]
        '''
        '''
        Convert the input integer to an 8-bit signed integer.
        input value range:[0,255]
        return Range: [-128, 127]
        '''
        # 範囲チェック
        if not (0 <= value <= 255):
            self.logger.error("Error ConvertToNegative_int8_t: Input value exceeds the range [0, 255].")
        # 8ビット符号なし整数に変換
        value &= 0xFF  # value を 8 ビット符号なし整数に変換
        if value & 0x80:  # 符号ビットをチェック
            value -= 0x100  # 符号ビットが 1 の場合、負数を表すため、2^8 を減算する必要があります
        return value

    def ConvertToNegative_uint8_t(self, value: int) -> int:
        '''
        入力された整数を8ビット符号なし整数に変換します。
        入力 value 範囲:[0,255]
        return 範囲: [0, 255]
        '''
        '''
        Convert the input integer to an 8-bit unsigned integer.
        input value range:[0,255]
        return Range: [0, 255]
        '''
        # 範囲チェック
        if not (0 <= value <= 255):
            self.logger.error("Error ConvertToNegative_uint8_t: Input value exceeds the range [0, 255].")
        # 8ビット符号なし整数に変換
        value &= 0xFF  # value を 8 ビット符号なし整数に変換
        return value

    def ConvertToNegative_16bit(self, value: int, signed:bool=True) -> int:
        '''
        入力された整数を16ビット整数に変換します。
        入力 value 範囲:[0,65535]
        signed が True の場合、16ビット符号付き整数に変換されます。[-32768, 32767]
        signed が False の場合、16ビット符号なし整数に変換されます。[0, 65535]
        '''
        '''
        Convert the input integer to a 16-bit integer.
        input value range:[0,65535]
        If signed is True, it will be converted to a 16-bit signed integer. Range: [-32768, 32767]
        If signed is False, it will be converted to a 16-bit unsigned integer. Range: [0, 65535]
        '''
        # 範囲チェック
        if not (0 <= value <= 65535):
            self.logger.error("Error ConvertToNegative_16bit: Input value exceeds the range [0, 65535].")
        # 16ビット符号なし整数に変換
        value &= 0xFFFF  # value を 16 ビット符号なし整数に変換
        if signed:
            if value & 0x8000:  # 符号ビットをチェック
                value -= 0x10000  # 符号ビットが 1 の場合、負数を表すため、2^16 を減算する必要があります
        return value
    
    def ConvertToNegative_int16_t(self, value: int) -> int:
        '''
        入力された整数を16ビット符号付き整数に変換します。
        入力 value 範囲:[0,65535]
        return 範囲: [-32768, 32767]
        '''
        '''
        Convert the input integer to a 16-bit signed integer.
        input value range:[0,65535]
        return Range: [-32768, 32767]
        '''
        # 範囲チェック
        if not (0 <= value <= 65535):
            self.logger.error("Error ConvertToNegative_int16_t: Input value exceeds the range [0, 65535].")
        # 16ビット符号なし整数に変換
        value &= 0xFFFF  # value を 16 ビット符号なし整数に変換
        if value & 0x8000:  # 符号ビットをチェック
            value -= 0x10000  # 符号ビットが 1 の場合、負数を表すため、2^16 を減算する必要があります
        return value

    def ConvertToNegative_uint16_t(self, value: int) -> int:
        '''
        入力された整数を16ビット符号なし整数に変換します。
        入力 value 範囲:[0,65535]
        return 範囲: [0, 65535]
        '''
        '''
        Convert the input integer to a 16-bit unsigned integer.
        input value range:[0,65535]
        return Range: [0, 65535]
        '''
        # 範囲チェック
        if not (0 <= value <= 65535):
            self.logger.error("Error ConvertToNegative_uint16_t: Input value exceeds the range [0, 65535].")
        # 16ビット符号なし整数に変換
        value &= 0xFFFF  # value を 16 ビット符号なし整数に変換
        return value

    def ConvertToNegative_32bit(self, value:int, signed:bool=True):
        '''
        入力された整数を32ビット整数に変換します。
        入力 value 範囲:[0,4294967295]
        signed が True の場合、32ビット符号付き整数に変換されます。
        signed が False の場合、32ビット符号なし整数に変換されます。
        '''
        '''
        Convert the input integer to a 32-bit integer.
        input value range:[0,4294967295]
        If signed is True, it will be converted to a 32-bit signed integer.
        If signed is False, it will be converted to a 32-bit unsigned integer.
        '''
        # 範囲チェック
        if not (0 <= value <= 4294967295):
            self.logger.error("Error ConvertToNegative_32bit: Input value exceeds the range [0, 4294967295].")
        # 32ビット符号なし整数に変換
        value &= 0xFFFFFFFF  # value を 32 ビット符号なし整数に変換
        if signed:
            if value & 0x80000000:  # 符号ビットをチェック
                value -= 0x100000000  # 符号ビットが 1 の場合、負数を表すため、2^32 を減算する必要があります
        return value
    
    def ConvertToNegative_int32_t(self, value: int) -> int:
        '''
        入力された整数を32ビット符号付き整数に変換します。
        入力 value 範囲:[0,4294967295]
        return範囲: [-2147483648, 2147483647]
        '''
        '''
        Convert the input integer to a 32-bit signed integer.
        input value range:[0,4294967295]
        return Range: [-2147483648, 2147483647].
        '''
        # 範囲チェック
        if not (0 <= value <= 4294967295):
            self.logger.error("Error ConvertToNegative_int32_t: Input value exceeds the range [0, 4294967295].")
        # 32ビット符号なし整数に変換
        value &= 0xFFFFFFFF  # value を 32 ビット符号なし整数に変換
        if value & 0x80000000:  # 符号ビットをチェック
            value -= 0x100000000  # 符号ビットが 1 の場合、負数を表すため、2^32 を減算する必要があります
        return value

    def ConvertToNegative_uint32_t(self, value: int) -> int:
        '''
        入力された整数を32ビット符号なし整数に変換します。
        入力 value 範囲:[0,4294967295]
        return範囲: [0, 4294967295]
        '''
        '''
        Convert the input integer to a 32-bit unsigned integer.
        input value range:[0,4294967295]
        return Range: [0, 4294967295].
        '''
        # 範囲チェック
        if not (0 <= value <= 4294967295):
            self.logger.error("Error ConvertToNegative_uint32_t: Input value exceeds the range [0, 4294967295].")
        # 32ビット符号なし整数に変換
        value &= 0xFFFFFFFF  # value を 32 ビット符号なし整数に変換
        return value

    def ConvertToList_8bit(self, value: int, signed: bool = True):
        '''
        入力された整数を8ビット整数リストに変換します。
        signedパラメータに基づいて、符号付き整数として扱うかどうかを判断します。
        範囲外の場合は警告を表示します。
        '''
        '''
        Convert the input integer into an 8-bit integer list.
        The signed parameter determines whether it is treated as a signed integer.
        A warning will be given if the value exceeds the allowed range.
        '''
        if signed:
            if not -128 <= value <= 127:
                raise OverflowError(f"The input value {value} exceeds the range of an 8-bit signed integer [-128, 127].")
            value = ctypes.c_int8(value).value
            return list(struct.unpack("B", struct.pack(">b", value)))
        else:
            if not 0 <= value <= 255:
                raise OverflowError(f"The input value {value} exceeds the range of an 8-bit unsigned integer [0, 255].")
            return list(struct.unpack("B", struct.pack(">B", value)))
    
    def ConvertToList_int8_t(self, value: int):
        if not -128 <= value <= 127:
            raise OverflowError(f"入力値 {value} は8ビット符号付き整数の範囲 [-128, 127] を超えています。")
        if value < 0:
            value = (value + 0x100) & 0xFF  # 8ビット表現に変換
        else:
            value &= 0xFF
        return [value]
    
    def ConvertToList_uint8_t(self, value: int):
        if not 0 <= value <= 255:
            raise OverflowError(f"入力値 {value} は8ビット符号なし整数の範囲 [0, 255] を超えています。")
        value &= 0xFF
        return [value]

    def ConvertToList_16bit(self, value: int, signed: bool = True):
        '''
        入力された整数を16ビット整数リストに変換します。
        signedパラメータに基づいて、符号付き整数として扱うかどうかを判断します。
        範囲外の場合は警告を表示します。
        '''
        '''
        Convert the input integer into a 16-bit integer list.
        The signed parameter determines whether it is treated as a signed integer.
        A warning will be given if the value exceeds the allowed range.
        '''
        if signed:
            if not -32768 <= value <= 32767:
                raise OverflowError(f"The input value {value} exceeds the range of a 16-bit signed integer [-32768, 32767].")
            value = ctypes.c_int16(value).value
            return list(struct.unpack("BB", struct.pack(">h", value)))
        else:
            if not 0 <= value <= 65535:
                raise OverflowError(f"The input value {value} exceeds the range of a 16-bit unsigned integer [0, 65535].")
            return list(struct.unpack("BB", struct.pack(">H", value)))

    def ConvertToList_int16_t(self, value: int):
        if not -32768 <= value <= 32767:
            raise OverflowError(f"入力値 {value} は16ビット符号付き整数の範囲 [-32768, 32767] を超えています。")
        if value < 0:
            value = (value + 0x10000) & 0xFFFF  # 16ビット表現に変換
        else:
            value &= 0xFFFF
        # 結果を2つのuint8値に分割
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        return [high_byte, low_byte]
    
    def ConvertToList_uint16_t(self, value: int):
        if not 0 <= value <= 65535:
            raise OverflowError(f"入力値 {value} は16ビット符号なし整数の範囲 [0, 65535] を超えています。")
        value &= 0xFFFF
        # 結果を2つのuint8値に分割
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        return [high_byte, low_byte]

    def ConvertToList_32bit(self, value: int, signed: bool = True):
        '''
        入力された整数を32ビット整数リストに変換します。
        signedパラメータに基づいて、符号付き整数として扱うかどうかを判断します。
        範囲外の場合は警告を表示します。
        '''
        '''
        Convert the input integer into a 32-bit integer list.
        The signed parameter determines whether it is treated as a signed integer.
        A warning will be given if the value exceeds the allowed range.
        '''
        if signed:
            if not -2147483648 <= value <= 2147483647:
                raise OverflowError(f"The input value {value} exceeds the range of a 32-bit signed integer [-2147483648, 2147483647].")
            value = ctypes.c_int32(value).value
            return list(struct.unpack("BBBB", struct.pack(">i", value)))
        else:
            if not 0 <= value <= 4294967295:
                raise OverflowError(f"The input value {value} exceeds the range of a 32-bit unsigned integer [0, 4294967295].")
            return list(struct.unpack("BBBB", struct.pack(">I", value)))

    def ConvertToList_int32_t(self, value: int):
        if not -2147483648 <= value <= 2147483647:
            raise OverflowError(f"入力値 {value} は32ビット符号付き整数の範囲 [-2147483648, 2147483647] を超えています。")
        if value < 0:
            value = (value + 0x100000000) & 0xFFFFFFFF  # 32ビット表現に変換
        else:
            value &= 0xFFFFFFFF
        # 結果を4つのuint8値に分割
        byte_3 = (value >> 24) & 0xFF
        byte_2 = (value >> 16) & 0xFF
        byte_1 = (value >> 8) & 0xFF
        byte_0 = value & 0xFF
        return [byte_3, byte_2, byte_1, byte_0]
    
    def ConvertToList_uint32_t(self, value: int):
        if not 0 <= value <= 4294967295:
            raise OverflowError(f"入力値 {value} は32ビット符号なし整数の範囲 [0, 4294967295] を超えています。")
        value &= 0xFFFFFFFF
        # 結果を4つのuint8値に分割
        byte_3 = (value >> 24) & 0xFF
        byte_2 = (value >> 16) & 0xFF
        byte_1 = (value >> 8) & 0xFF
        byte_0 = value & 0xFF
        return [byte_3, byte_2, byte_1, byte_0]

    def FloatToUint(self,x_float:float, x_min:float, x_max:float, bits:int):
        '''
        浮動小数点数を符号なし整数に変換
        MITモードのパススルー制御で個々の関節モータードライバモードで使用
        '''
        '''
        Convert floating point number to unsigned integer
        Used in MIT mode pass-through control for individual joint motor driver mode
        '''
        span:float = x_max - x_min
        offset:float = x_min
        return int((x_float - offset) * (float((1<<bits)-1))/span)
    
    def ConvertBytesToInt(self, bytes:bytearray, first_index:int, second_index:int, byteorder:Literal["little", "big"]='big'):
        '''
        バイト列をint型に変換、デフォルトはビッグエンディアン
        '''
        '''
        Convert a byte string to an int type, with big-endian byte order by default.
        '''
        return int.from_bytes(bytes[first_index:second_index], byteorder=byteorder)