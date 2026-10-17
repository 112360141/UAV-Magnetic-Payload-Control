# --- ⚠️ 補丁區 (解決 Python 3.10+ 相容性問題) ---
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping
# ----------------

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import cv2
import time
import numpy as np

# ==========================================
#  使用者設定區
# ==========================================
target_com_port = 'COM3'   # 請確認您的 COM 埠 (例如 /dev/ttyACM0 或 COM3)
target_baud_rate = 57600   
SERVO_OUTPUT_PIN = 5       # 對應 Pixhawk 的 Main Out 5
# ==========================================

print("---------------------------------------")
print(f"正在連線至: {target_com_port}...")
try:
    # wait_ready=False 先建立連線，稍後再等待參數讀取
    vehicle = connect(target_com_port, baud=target_baud_rate, wait_ready=False, heartbeat_timeout=60)
    print(">>> 連線成功！正在讀取參數(請稍候)...")
    vehicle.wait_ready(True, timeout=300)
except Exception as e:
    print(f"連線失敗: {e}")
    exit()

print(f"✅ 系統就緒！模式: {vehicle.mode.name}")

# --- 功能函式：控制實體 PWM 輸出腳位 ---
def set_servo(servo_number, pwm_value):
    """ 
    直接控制 Pixhawk 上的實體腳位 (Main Out)
    servo_number: 腳位編號 (5 代表 Main Out 5)
    pwm_value: 1000~2000 之間的數值
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # 指令
        0,       # confirmation
        servo_number, # 參數 1: 腳位
        pwm_value,    # 參數 2: PWM 值
        0, 0, 0, 0, 0 # 參數 3~7
    )
    vehicle.send_mavlink(msg)
    print(f">>> [MAVLink] 已發送指令至 Main Out {servo_number}: PWM {pwm_value}")

# --- 室內解鎖函式 ---
def arm_for_indoor_test():
    print("⚠️ 執行室內解鎖...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)
    vehicle.armed = True
    
    timeout = 0
    while not vehicle.armed:
        print("   等待解鎖...")
        time.sleep(1)
        timeout += 1
        if timeout > 5:
            print("❌ 解鎖失敗 (請檢查 Mission Planner 安全設定或 Pre-arm Check)")
            return
    print("✅ 馬達已解鎖！")

# --- 清除所有覆寫控制 ---
def clear_all_overrides():
    vehicle.channels.overrides = {}

# --- 介面說明 ---
print("---------------------------------------")
print("【 無人機投擲控制系統 - Pixhawk Main Out 版 】")
print(" SPACE : 解鎖馬達")
print(" M     : 🧲 切換電磁鐵 (Main Out 5)")
print(" W/S   : 油門 (上升/下降)")
print(" I/K   : 前進/後退")
print(" J/L   : 左移/右移")
print(" A/D   : 左轉/右轉")
print(" Z     : 自動降落")
print(" Q     : 緊急停止/結束程式")
print("---------------------------------------")

# 初始化 OpenCV 視窗
img = np.zeros((350, 450, 3), dtype=np.uint8)

# --- 變數初始化 ---
throttle_pwm = 1000 
THROTTLE_STEP = 30
THROTTLE_MAX = 1800 
THROTTLE_MIN = 1000 
PITCH_ROLL_STEP = 200 
YAW_STEP = 200

# 🧲 電磁鐵狀態
magnet_is_on = False 
magnet_pwm_high = 0 # 開啟時的 PWM
magnet_pwm_low = 1900  # 關閉時的 PWM

is_auto_landing = False

try:
    while True:
        # --- UI 繪製 ---
        img.fill(0) 
        cv2.putText(img, f"Mode: {vehicle.mode.name}", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1)
        cv2.putText(img, f"Throttle: {throttle_pwm}", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 顯示磁鐵狀態 (綠色=開, 紅色=關)
        if magnet_is_on:
            cv2.putText(img, "MAGNET: ON (Main 5)", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(img, f"PWM: {magnet_pwm_high}", (30, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        else:
            cv2.putText(img, "MAGNET: OFF (Main 5)", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img, f"PWM: {magnet_pwm_low}", (30, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            
        cv2.imshow('Controller', img)

        key = cv2.waitKey(100) & 0xFF 
        
        # 歸零方向 PWM
        roll_pwm = 1500
        pitch_pwm = 1500
        yaw_pwm = 1500
        has_input = False 

        # --- 按鍵邏輯 ---
        if key == ord('q'):
            print("程式結束。")
            vehicle.armed = False
            break

        elif key == 32: # Space
            arm_for_indoor_test()

        # --- 🧲 磁鐵切換 (直接驅動 Main Out 5) ---
        elif key == ord('m'):
            magnet_is_on = not magnet_is_on 
            pwm_to_send = magnet_pwm_high if magnet_is_on else magnet_pwm_low
            set_servo(SERVO_OUTPUT_PIN, pwm_to_send)
            has_input = True

        # --- 飛行控制 ---
        elif key == ord('z'): 
            is_auto_landing = True
            print(">>> 啟動自動降落模式")

        elif key == ord('w'): 
            is_auto_landing = False
            throttle_pwm = min(throttle_pwm + THROTTLE_STEP, THROTTLE_MAX)
            has_input = True

        elif key == ord('s'): 
            is_auto_landing = False
            throttle_pwm = max(throttle_pwm - THROTTLE_STEP, THROTTLE_MIN)
            has_input = True

        # 方向控制 (I/K/J/L/A/D)
        if key == ord('i'): pitch_pwm = 1500 - PITCH_ROLL_STEP; has_input = True
        elif key == ord('k'): pitch_pwm = 1500 + PITCH_ROLL_STEP; has_input = True
        if key == ord('j'): roll_pwm = 1500 - PITCH_ROLL_STEP; has_input = True
        elif key == ord('l'): roll_pwm = 1500 + PITCH_ROLL_STEP; has_input = True
        if key == ord('a'): yaw_pwm = 1500 - YAW_STEP; has_input = True
        elif key == ord('d'): yaw_pwm = 1500 + YAW_STEP; has_input = True

        # 自動降落邏輯
        if is_auto_landing:
            throttle_pwm -= 20
            if throttle_pwm <= THROTTLE_MIN:
                throttle_pwm = THROTTLE_MIN
                vehicle.armed = False
                is_auto_landing = False
                print(">>> 降落完成，馬達已上鎖")
            has_input = True

        # --- 發送 RC 指令 (覆寫模式) ---
        # 注意：我們只覆寫 1-4 通道，磁鐵已經由 set_servo 獨立控制了
        if vehicle.armed or has_input:
            vehicle.channels.overrides = {
                '1': roll_pwm,      
                '2': pitch_pwm,     
                '3': throttle_pwm,  
                '4': yaw_pwm
            }
        else:
            vehicle.channels.overrides = {}

except KeyboardInterrupt:
    print("\n使用者中斷程式")

finally:
    print("正在清理資源並關閉連線...")
    clear_all_overrides()
    cv2.destroyAllWindows()
    if vehicle:
        vehicle.close()
    print("Done.")