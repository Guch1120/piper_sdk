import threading
import time
from collections import defaultdict
from collections import deque

class C_FPSCounter:
    def __init__(self,
                 start_realtime_fps:bool = False):
        """ FPS統計カウンターを初期化 """
        self.start_realtime_fps = start_realtime_fps
        self.fps_data = defaultdict(int)  # フレームカウントを記録
        self.fps_results = defaultdict(float)  # 計算されたFPS結果
        self.prev_data = defaultdict(int)  # 前回のカウント値
        if(self.start_realtime_fps):
            self.time_stamps = defaultdict(deque)  # タイムスタンプを保存
        self.last_time = defaultdict(float)  # 前回のフレーム時刻を記録
        self.lock = threading.Lock()  # スレッドセーフを確保
        self.running = False  # スレッド状態を制御
        self.thread = None  # スレッドオブジェクト
        self.stop_event = threading.Event()  # スレッド終了制御用
        self.__interval = 0.1

    def set_cal_fps_time_interval(self, interval:float=0.1):
        self.__interval = interval
        return self.__interval
    
    def get_cal_fps_time_interval(self):
        return self.__interval

    def add_variable(self, name, window_size=5000):
        """ 新しいFPS変数を追加し、時間ウィンドウサイズを制限 """
        with self.lock:
            if name not in self.fps_data:
                self.fps_data[name] = 0
                self.fps_results[name] = 0.0
                if(self.start_realtime_fps):
                    self.time_stamps[name] = deque(maxlen=window_size)  # 最大保存ウィンドウを制限
                self.last_time[name] = time.perf_counter()

    def increment(self, name):
        """ フレームカウントをインクリメントし、タイムスタンプを記録 """
        current_time = time.perf_counter()
        with self.lock:
            if name in self.fps_data:
                self.fps_data[name] += 1
                if(self.start_realtime_fps):
                    self.time_stamps[name].append(current_time)  # `deque`が自動的に期限切れデータを管理
                self.last_time[name] = current_time

    def get_fps(self, name):
        """ 1秒以内のFPS計算結果を取得 """
        multiple = 1 / self.__interval
        with self.lock:
            return self.fps_results.get(name, 0.0) * multiple

    def get_real_time_fps(self, name, window=1.0):
        """ 過去 window 秒のリアルタイムFPSを計算 """
        now = time.perf_counter()
        with self.lock:
            if(self.start_realtime_fps):
                while self.time_stamps[name] and now - self.time_stamps[name][0] > window:
                    self.time_stamps[name].popleft()  # 最も古いタイムスタンプを直接破棄
                
                return len(self.time_stamps[name]) / window if self.time_stamps[name] else 0.0
            else: return -1

    def start(self):
        """ FPS計算スレッドを起動、重複起動を防止 """
        with self.lock:
            if self.running:
                return  # 既に実行中、重複起動を回避
            self.running = True
            self.stop_event.clear()
        
        self.thread = threading.Thread(target=self._calculate_fps, daemon=True)
        self.thread.start()

    def stop(self):
        """ FPS計算スレッドを停止 """
        with self.lock:
            if not self.running:
                return  # 既に停止済み
            self.running = False
            self.stop_event.set()  # イベントを設定し、スレッド終了を確保
        
        if self.thread and self.thread.is_alive():
            self.thread.join()

    def cal_average(self, *args):
        """ 一組の数の平均値を計算、ただしすべての数が0でない場合のみ計算；0がある場合は直接0を返す """
        return round(sum(args) / len(args) if args and all(args) else 0,3)

    def _calculate_fps(self):
        """ 定期的にFPSを計算 """
        while not self.stop_event.is_set():
            with self.lock:
                for name in self.fps_data:
                    self.fps_results[name] = self.fps_data[name] - self.prev_data[name]
                    self.prev_data[name] = self.fps_data[name]
            self.stop_event.wait(self.__interval)  # wait()をsleep()の代わりに使用、制御が容易

# def camera_simulation(fps_counter, name, interval, stop_after=None):
#     """
#     模拟相机帧生成并调用 increment
#     :param fps_counter: FPSCounter 实例
#     :param name: 相机变量名称
#     :param interval: 模拟帧生成的时间间隔
#     :param stop_after: 停止生成数据的时间（秒），为 None 时不停止
#     """
#     start_time = time.perf_counter()
#     while True:
#         if stop_after is not None and time.perf_counter() - start_time > stop_after:
#             print(f"{name} stopped generating data.")
#             break
#         fps_counter.increment(name)
#         time.sleep(interval)


# if __name__ == "__main__":
#     fps_counter = C_FPSCounter()
#     fps_counter.add_variable("camera1")
#     fps_counter.add_variable("camera2")

#     fps_counter.start()

#     try:
#         # 创建线程模拟相机1和相机2的帧生成
#         camera1_thread = threading.Thread(target=camera_simulation, args=(fps_counter, "camera1", 0.01, 5), daemon=True)
#         camera2_thread = threading.Thread(target=camera_simulation, args=(fps_counter, "camera2", 0.01, None), daemon=True)

#         camera1_thread.start()
#         camera2_thread.start()

#         # 打印 FPS
#         while True:
#             print(f"FPS for camera1 (1s avg): {fps_counter.get_fps('camera1')}")
#             print(f"FPS for camera2 (1s avg): {fps_counter.get_fps('camera2')}")
#             print(f"Real-time FPS for camera1: {fps_counter.get_real_time_fps('camera1', window=1.0)}")
#             print(f"Real-time FPS for camera2: {fps_counter.get_real_time_fps('camera2', window=1.0)}")
#             print(f"Instant FPS for camera1: {fps_counter.get_instant_fps('camera1')}")
#             print(f"Instant FPS for camera2: {fps_counter.get_instant_fps('camera2')}")
#             time.sleep(0.01)

#     finally:
#         fps_counter.stop()
