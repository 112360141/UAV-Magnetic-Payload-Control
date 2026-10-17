
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


target_com_port = 'COM3'   
target_baud_rate = 57600   
SERVO_OUTPUT_PIN = 5       

print("---------------------------------------")
print(f"正在連線至: {target_com_port}...")
try:
    vehicle = connect(target_com_port, baud=target_baud_rate, wait_ready=False, heartbeat_timeout=60)
    print(">>> 連線成功！正在讀取參數(請稍候)...")
    vehicle.wait_ready(True, timeout=300)
except Exception as e:
    print(f"連線失敗: {e}")
    exit()

print(f"✅ 系統就緒！模式: {vehicle.mode.name}")

def set_servo(servo_number, pwm_value):
    """ 
    直接控制 Pixhawk 上的實體腳位 (Main Out)
    servo_number: 腳位編號 (5 代表 Main Out 5)
    pwm_value: 1000~2000 之間的數值
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,   
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
        0,       
        servo_number, 
        pwm_value,    
        0, 0, 0, 0, 0 
    )
    vehicle.send_mavlink(msg)
    print(f">>> [MAVLink] 已發送指令至 Main Out {servo_number}: PWM {pwm_value}")

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

def clear_all_overrides():
    vehicle.channels.overrides = {}

# --- 介面說明 ---
print("---------------------------------------")
print("【 無人機投擲控制系統 - Pixhawk Main Out 版 】")
print(" SPACE : 解鎖馬達")
print(" M     :  切換電磁鐵 (Main Out 5)")
print(" W/S   : 油門 (上升/下降)")
print(" I/K   : 前進/後退")
print(" J/L   : 左移/右移")
print(" A/D   : 左轉/右轉")
print(" Z     : 自動降落")
print(" Q     : 緊急停止/結束程式")
print("---------------------------------------")

img = np.zeros((350, 450, 3), dtype=np.uint8)

throttle_pwm = 1000 
THROTTLE_STEP = 30
THROTTLE_MAX = 1800 
THROTTLE_MIN = 1000 
PITCH_ROLL_STEP = 200 
YAW_STEP = 200

magnet_is_on = False 
magnet_pwm_high = 0 
magnet_pwm_low = 1900  

is_auto_landing = False

try:
    while True:
        img.fill(0) 
        cv2.putText(img, f"Mode: {vehicle.mode.name}", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1)
        cv2.putText(img, f"Throttle: {throttle_pwm}", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        if magnet_is_on:
            cv2.putText(img, "MAGNET: ON (Main 5)", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(img, f"PWM: {magnet_pwm_high}", (30, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        else:
            cv2.putText(img, "MAGNET: OFF (Main 5)", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img, f"PWM: {magnet_pwm_low}", (30, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            
        cv2.imshow('Controller', img)

        key = cv2.waitKey(100) & 0xFF 
        
        roll_pwm = 1500
        pitch_pwm = 1500
        yaw_pwm = 1500
        has_input = False 

        if key == ord('q'):
            print("程式結束。")
            vehicle.armed = False
            break

        elif key == 32: 
            arm_for_indoor_test()

        elif key == ord('m'):
            magnet_is_on = not magnet_is_on 
            pwm_to_send = magnet_pwm_high if magnet_is_on else magnet_pwm_low
            set_servo(SERVO_OUTPUT_PIN, pwm_to_send)
            has_input = True

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

        if key == ord('i'): pitch_pwm = 1500 - PITCH_ROLL_STEP; has_input = True
        elif key == ord('k'): pitch_pwm = 1500 + PITCH_ROLL_STEP; has_input = True
        if key == ord('j'): roll_pwm = 1500 - PITCH_ROLL_STEP; has_input = True
        elif key == ord('l'): roll_pwm = 1500 + PITCH_ROLL_STEP; has_input = True
        if key == ord('a'): yaw_pwm = 1500 - YAW_STEP; has_input = True
        elif key == ord('d'): yaw_pwm = 1500 + YAW_STEP; has_input = True

        if is_auto_landing:
            throttle_pwm -= 20
            if throttle_pwm <= THROTTLE_MIN:
                throttle_pwm = THROTTLE_MIN
                vehicle.armed = False
                is_auto_landing = False
                print(">>> 降落完成，馬達已上鎖")
            has_input = True

   
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
