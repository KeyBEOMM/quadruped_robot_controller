#!/usr/bin/env python3
"""
Phase 4 Verification Test — PC-side UDP sender
=====================================================
CommTask UDP packet format (28 bytes, little-endian):
  [0..3]   float  vx
  [4..7]   float  vy
  [8..11]  float  wz
  [12..15] float  roll
  [16..19] float  pitch
  [20..23] float  yaw
  [24]     uint8  flags  (bit0 = reset_requested)
  [25..27] uint8  padding

Usage:
  python3 phase4_test.py --ip <ESP32_IP>

Test sequence (Core 0 + Core 1 integrated):
  test1 (5s) : vx 0.1→0.59 (0.1s마다 +0.01) → IK 각도 범위(5~175 deg) 확인
  test2 (3s) : wz=0.5  → T_cycle 0.4~0.8s 범위 확인
  test3 (2s) : 전송 중단 → Watchdog TIMEOUT→ERROR 전이 확인 (100ms 이내)
  reset      : flags bit0=1 → ERROR→INIT 복귀 확인
"""

import socket
import struct
import time
import argparse
import sys

# ─────────────────────────────────────────────
# 설정
# ─────────────────────────────────────────────
UDP_PORT       = 9870
PACKET_LEN     = 28
SEND_RATE_HZ   = 50          # 50Hz = 20ms 간격 (제어 주기와 동일)
SEND_INTERVAL  = 1.0 / SEND_RATE_HZ


def build_packet(vx=0.0, vy=0.0, wz=0.0,
                 roll=0.0, pitch=0.0, yaw=0.0,
                 reset=False) -> bytes:
    """28바이트 UDP 명령 패킷을 빌드한다."""
    flags = 0x01 if reset else 0x00
    payload = struct.pack('<ffffffBxxx',
                          vx, vy, wz,
                          roll, pitch, yaw,
                          flags)
    assert len(payload) == PACKET_LEN, f"Packet len mismatch: {len(payload)}"
    return payload


def send_for(sock, addr, duration_s: float,
             vx=0.0, vy=0.0, wz=0.0,
             roll=0.0, pitch=0.0, yaw=0.0,
             label=""):
    """지정된 duration(초) 동안 50Hz로 패킷을 전송한다."""
    pkt = build_packet(vx=vx, vy=vy, wz=wz,
                       roll=roll, pitch=pitch, yaw=yaw)
    end = time.time() + duration_s
    count = 0
    while time.time() < end:
        t_start = time.time()
        sock.sendto(pkt, addr)
        count += 1
        elapsed = time.time() - t_start
        sleep_t = SEND_INTERVAL - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)
    print(f"  [{label}] Sent {count} packets over {duration_s}s "
          f"(vx={vx:.2f} vy={vy:.2f} wz={wz:.2f})")


def main():
    parser = argparse.ArgumentParser(description="Phase 4 UDP Test Client")
    parser.add_argument("--ip",  required=True, help="ESP32 IP address")
    parser.add_argument("--port", type=int, default=UDP_PORT)
    args = parser.parse_args()

    addr = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)

    print("=" * 50)
    print("[Phase 4 Test] PC-side UDP sender")
    print(f"  Target : {addr[0]}:{addr[1]}")
    print(f"  Rate   : {SEND_RATE_HZ} Hz")
    print("=" * 50)
    print()

    # ─────────────────────────────────────────────
    # Warm-up (1s)
    #   로봇이 INIT -> IDLE 상태로 전환할 시간을 주고,
    #   Watchdog을 리셋하는 속도 0 패킷을 보낸다.
    # ─────────────────────────────────────────────
    print("[Warm-up] Ensuring robot is in IDLE state...")
    send_for(sock, addr, duration_s=1.0, vx=0.0, label="warm-up")
    print()

    # ─────────────────────────────────────────────
    # test1: vx 선형 증가 (0.1초마다 +0.01, 총 5초)
    #   0.0s: vx=0.10 / 0.1s: vx=0.11 / ... / 4.9s: vx=0.59
    # 검증: 전 구간에서 ESP32 시리얼에 [WARN] 없으면 PASS
    # ─────────────────────────────────────────────
    VX_START  = 0.10   # 초기 vx (m/s)
    VX_STEP   = 0.01   # 0.1초마다 증가량
    VX_HOLD   = 0.10   # 각 단계 유지 시간 (s)
    STEP_COUNT = 50    # 총 단계 수 (= 5s / 0.1s)

    print("[test1] vx ramp: 0.10 -> 0.59 (+0.01 per 0.1s, 5s total)")
    print("        (ESP32 serial: IK[LF]: ... every 1s, NO [WARN] = PASS)")

    total_sent = 0
    for step in range(STEP_COUNT):
        vx_now = VX_START + step * VX_STEP
        pkt = build_packet(vx=vx_now)
        t_end = time.time() + VX_HOLD
        step_count = 0
        while time.time() < t_end:
            t0 = time.time()
            sock.sendto(pkt, addr)
            step_count += 1
            elapsed = time.time() - t0
            slp = SEND_INTERVAL - elapsed
            if slp > 0:
                time.sleep(slp)
        total_sent += step_count  # type: ignore
        print(f"  [test1] step {step+1:02d}/{STEP_COUNT} | vx={vx_now:.2f} | sent {step_count} pkts")
    print(f"  [test1] DONE — total {total_sent} packets")
    print()

    # ─────────────────────────────────────────────
    # test2: wz=0.5 제자리 회전 3초
    # 검증: ESP32 시리얼에 T_cycle=X.XXXs [PASS] 출력 → PASS
    # ─────────────────────────────────────────────
    print("[test2] wz=0.5 for 3s  ->  Check T_cycle in 0.4~0.8s range")
    print("        (ESP32 serial: 'T_cycle=X.XXXs [PASS]' every 0.5s)")
    send_for(sock, addr, duration_s=3.0, wz=0.5, label="test2")
    print()

    # ─────────────────────────────────────────────
    # test3: 전송 중단 (2초) → Watchdog TIMEOUT 확인
    # 검증: 100ms 이내에 '[Watchdog] TIMEOUT -> ERROR' 로그 출력 → PASS
    # ─────────────────────────────────────────────
    print("[test3] Stop sending for 2s  ->  Watchdog TIMEOUT -> ERROR")
    print("        (ESP32 serial: '[Watchdog] TIMEOUT ... -> ERROR' within 100ms = PASS)")
    time.sleep(2.0)
    print("  [test3] 2s silence done")
    print()

    # ─────────────────────────────────────────────
    # reset: flags bit0=1 → ERROR -> INIT 복귀
    # 검증: '[Transition] ERROR -> INIT' 로그 출력 → PASS
    # ─────────────────────────────────────────────
    print("[reset] Sending reset command  ->  ERROR -> INIT transition")
    print("        (ESP32 serial: '[Transition] ERROR -> INIT' = PASS)")
    reset_pkt = build_packet(reset=True)
    for _ in range(5):   # 5회 연속 전송 (확실한 수신 보장)
        sock.sendto(reset_pkt, addr)
        time.sleep(0.02)
    print("  [reset] Reset packets sent")
    print()

    # ─────────────────────────────────────────────
    # 결과 요약
    # ─────────────────────────────────────────────
    print("=" * 50)
    print("[Phase 4 Test] ALL SENT")
    print()
    print("  Check ESP32 serial output for:")
    print("  test1: 'IK[LF]: ...' entries, NO [WARN]          -> PASS")
    print("  test2: 'T_cycle=X.XXXs [PASS]' every 0.5s       -> PASS (if guide.md 未변경)")
    print("  test3: '[Watchdog] TIMEOUT' within 100ms silence -> PASS")
    print("  reset: '[Transition] ERROR -> INIT'              -> PASS")
    print("=" * 50)

    # ─────────────────────────────────────────────
    # 보너스: Rising/Falling Edge 로그 확인 안내
    # ─────────────────────────────────────────────
    print()
    print("[Edge/Latch log check] During test1, also verify in serial:")
    print("  '[Leg0] STANCE->SWING Latched'  (Rising Edge detected)")
    print("  '[Leg0] SWING->STANCE Latched'  (Falling Edge detected)")
    print("  All 4 legs should appear alternately.")

    sock.close()


if __name__ == "__main__":
    main()
