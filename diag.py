"""
diag.py v4 — Force 모드 전환 (Phase 4) + I2C 테스트 + 실시간 모니터링
실행: python diag.py

1. CMD 0x06 (I2C Test)  → MCU printf로 I2C 결과 즉시 출력
2. CMD 0x01 mode[CH]=CH_FORCE, target=100g
   → MCU가 Force 모드로 전환되어 CMD 0x05 프레임 전송 시작
   (Phase 4: CMD 0x04 폐기, CMD 0x01에 통합)
Ctrl+C로 종료
"""

import time
import struct
import serial

PORT     = "COM6"
BAUDRATE = 115200

STX               = 0xAA
CMD_CONTROL       = 0x01
CMD_STATE         = 0x02
CMD_FORCE_STATE   = 0x05
CMD_I2C_TEST      = 0x06

CTRL_CH = 6

# Channel control modes (Phase 4)
CH_OFF    = 0
CH_MANUAL = 1
CH_TEMP   = 2
CH_FORCE  = 3

FORCE_CHANNEL     = 0       # 힘 제어 채널
FORCE_TARGET      = 100.0   # 목표 힘 (g)


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes = b'') -> bytes:
    length   = len(payload)
    crc_data = bytes([cmd, length]) + payload
    return bytes([STX, cmd, length]) + payload + bytes([crc8(crc_data)])


def main():
    print(f"Connecting to {PORT} @ {BAUDRATE}baud...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] {e}")
        return

    print("Connected!\n")
    time.sleep(0.3)

    # ── 1. I2C 테스트 전송 ──
    f = build_frame(CMD_I2C_TEST)
    ser.write(f)
    print(f">> [1] CMD 0x06 I2C Test 전송: {f.hex()}")

    time.sleep(0.2)

    # ── 2. Force Control 활성화 (Phase 4: CMD 0x01 mode=CH_FORCE) ──
    # payload (30B): mode[6] + manual_pwm[6] + manual_fan[6] + target[6×u16]
    mode_arr       = [CH_OFF] * CTRL_CH
    manual_pwm_arr = [0]      * CTRL_CH
    manual_fan_arr = [0]      * CTRL_CH
    target_raw     = [0]      * CTRL_CH

    mode_arr[FORCE_CHANNEL]   = CH_FORCE
    target_raw[FORCE_CHANNEL] = max(0, min(0xFFFF, int(round(FORCE_TARGET * 10))))  # 0.1g/LSB

    payload = bytearray()
    payload.extend(mode_arr)
    payload.extend(manual_pwm_arr)
    payload.extend(manual_fan_arr)
    for v in target_raw:
        payload.append(v & 0xFF)
        payload.append((v >> 8) & 0xFF)

    f = build_frame(CMD_CONTROL, bytes(payload))
    ser.write(f)
    print(f">> [2] CMD 0x01 mode[{FORCE_CHANNEL}]=CH_FORCE  "
          f"Target={FORCE_TARGET}g  전송: {f.hex()}")
    print("\n실시간 수신 (Ctrl+C로 종료):\n")

    # ── 파서 ──
    state    = 'IDLE'
    cmd      = 0
    length   = 0
    data_buf = bytearray()
    text_buf = bytearray()
    counts   = {CMD_STATE: 0, CMD_FORCE_STATE: 0}

    try:
        while True:
            raw = ser.read(1)
            if not raw:
                continue
            byte = raw[0]

            if state == 'IDLE':
                if byte == STX:
                    if text_buf:
                        text = text_buf.decode('ascii', errors='replace').strip()
                        if text:
                            print(f"  [MCU printf] {text}")
                        text_buf = bytearray()
                    state = 'WAIT_CMD'
                else:
                    text_buf.append(byte)
                    if byte == ord('\n'):
                        text = text_buf.decode('ascii', errors='replace').strip()
                        if text:
                            print(f"  [MCU printf] {text}")
                        text_buf = bytearray()

            elif state == 'WAIT_CMD':
                cmd   = byte
                state = 'WAIT_LEN'

            elif state == 'WAIT_LEN':
                length   = byte
                data_buf = bytearray()
                if length == 0:
                    state = 'WAIT_CRC'
                elif length > 250:
                    state = 'IDLE'
                else:
                    state = 'WAIT_DATA'

            elif state == 'WAIT_DATA':
                data_buf.append(byte)
                if len(data_buf) >= length:
                    state = 'WAIT_CRC'

            elif state == 'WAIT_CRC':
                crc_ok = (byte == crc8(bytes([cmd, length]) + bytes(data_buf)))
                if not crc_ok:
                    state = 'IDLE'
                    continue

                if cmd == CMD_STATE and len(data_buf) >= 24:
                    counts[CMD_STATE] += 1
                    pwm  = data_buf[0]
                    fan  = data_buf[6]
                    temp = (data_buf[12] | (data_buf[13] << 8)) / 4.0
                    # 온도 프레임은 10개마다 한 번만 출력 (화면 넘침 방지)
                    if counts[CMD_STATE] % 10 == 1:
                        print(f"  [TEMP]  Temp={temp:.1f}°C  PWM={pwm}%  "
                              f"FAN={'ON' if fan else 'OFF'}  "
                              f"(rx#{counts[CMD_STATE]})")

                elif cmd == CMD_FORCE_STATE and len(data_buf) >= 10:
                    counts[CMD_FORCE_STATE] += 1
                    mode  = data_buf[0]
                    ch    = data_buf[1]
                    force, disp = struct.unpack_from('<ff', data_buf, 2)
                    # 힘 프레임은 매번 출력
                    print(f"  [FORCE] Force={force:.3f}g  Disp={disp:.3f}mm  "
                          f"Mode={mode}  CH={ch}  (rx#{counts[CMD_FORCE_STATE]})")

                state = 'IDLE'

    except KeyboardInterrupt:
        print(f"\n--- 수신 통계 ---")
        print(f"  온도 프레임  (0x02): {counts[CMD_STATE]}개")
        print(f"  힘   프레임  (0x05): {counts[CMD_FORCE_STATE]}개")
        if counts[CMD_FORCE_STATE] == 0:
            print("\n  ⚠️  힘 프레임 없음 — 확인사항:")
            print("  1. MCU printf에 'WARN' 메시지 (force_count > 1) 있는지 확인")
            print("  2. CMD 0x01이 MCU에 도달했는지 → UART RX 방향 / 페이로드 30B 확인")
            print("  3. mode[CH_FORCE 채널] 값이 3인지 확인")
            print("  4. I2C 문제 → 아두이노 배선/주소(0x08) 확인")

    ser.close()


if __name__ == "__main__":
    main()