import serial
import time

# 시리얼 포트 설정
port = '/dev/ttyACM0'  # 사용하는 포트에 맞게 수정하세요 (예: 'COM3', '/dev/ttyUSB0')
baudrate = 57600  # 보드레이트 설정

# 시리얼 객체 생성
ser = serial.Serial(port, baudrate, timeout=1)

# 잠시 대기 (연결이 안정화될 때까지)
time.sleep(2)

try:
    while True:
        # 데이터 읽기
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            print(f"받은 데이터: {data}")

except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    ser.close()
