# CAN ID 정의
RX_ID_STATE = 0x401  # MCU 상태 수신 (온도, 현재 PWM 등)
TX_ID_CTRL = 0x400   # 제어 명령 송신 (목표온도, PWM 설정 등)

# 시스템 설정
CTRL_CH = 6          # 전체 채널 수 (프로토콜 규격)