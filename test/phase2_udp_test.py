#!/usr/bin/env python3
"""
Phase 2 UDP 통신 검증 테스트 스크립트
======================================
사용법:
    python3 test/phase2_udp_test.py --ip <ESP32_IP주소>

    예: python3 test/phase2_udp_test.py --ip 192.168.1.105

시리얼 모니터에서 각 테스트 합격 기준 로그를 확인한다.
"""

import socket
import struct
import time
import argparse

# ─── UDP 패킷 포맷 (28바이트 고정, 리틀엔디언) ─────────────────
# [0..3]   float vx     [4..7]  float vy    [8..11]  float wz
# [12..15] float roll   [16..19] float pitch [20..23] float yaw
# [24]     uint8 flags (bit0 = reset_requested)
# [25..27] uint8 padding × 3
# ────────────────────────────────────────────────────────────────
UDP_PORT     = 9870
PACKET_FMT   = "<ffffffBxxx"   # 6× float + 1× uint8 + 3× padding
PACKET_BYTES = struct.calcsize(PACKET_FMT)  # = 28


def make_packet(vx=0.0, vy=0.0, wz=0.0,
                roll=0.0, pitch=0.0, yaw=0.0,
                reset=False) -> bytes:
    flags = 0x01 if reset else 0x00
    return struct.pack(PACKET_FMT, vx, vy, wz, roll, pitch, yaw, flags)


def send(sock, addr, pkt: bytes):
    sock.sendto(pkt, addr)


def separator(title: str):
    print(f"\n{'='*55}")
    print(f"  {title}")
    print(f"{'='*55}")


def main():
    parser = argparse.ArgumentParser(description="Phase 2 UDP Test")
    parser.add_argument("--ip",   required=True,          help="ESP32 IP Address")
    parser.add_argument("--port", type=int, default=9870, help="UDP Port (Default: 9870)")
    args = parser.parse_args()

    addr = (args.ip, args.port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(2.0)

        # ─────────────────────────────────────────────────────────
        # Test 1: Single packet transmission - Parsing log check
        # ─────────────────────────────────────────────────────────
        separator("Test 1: Single packet transmission - Parsing log check")
        print("► TX: vx=0.500, vy=0.100, wz=0.200, roll=0.050, pitch=0.010")
        pkt = make_packet(vx=0.5, vy=0.1, wz=0.2, roll=0.05, pitch=0.01)
        send(sock, addr, pkt)
        print("✔ TX completed.")
        print("  [ESP32 Serial] Check 'RX vx=0.500 vy=0.100 wz=0.200' log.")
        time.sleep(1.0)

        # ─────────────────────────────────────────────────────────
        # Test 2: 100Hz continuous TX for 5s - Deadlock check
        # ─────────────────────────────────────────────────────────
        separator("Test 2: 100Hz continuous TX for 5s - Deadlock check")
        count = 0
        t_end = time.time() + 5.0
        pkt = make_packet(vx=0.3, vy=0.0, wz=0.1)
        print("► Transmitting at 100Hz for 5 seconds... (Ctrl+C to abort)")
        while time.time() < t_end:
            send(sock, addr, pkt)
            count += 1
            time.sleep(0.01)  # 100Hz
        print(f"✔ {count} packets transmitted.")
        print("  [ESP32 Serial] Check for continuous logs without Panic/Exception.")
        time.sleep(0.5)

        # ─────────────────────────────────────────────────────────
        # Test 3: TX halt -> Watchdog ERROR (after 100ms)
        # ─────────────────────────────────────────────────────────
        separator("Test 3: TX halt -> Watchdog ERROR (after 100ms)")
        print("► Stopping packet TX. Waiting 150ms...")
        time.sleep(0.5)  # 여유 있게 0.5초 대기
        print("✔ Wait completed.")
        print("  [ESP32 Serial] Check '[Watchdog] TIMEOUT (...ms > 100ms) -> ERROR' log.")
        time.sleep(1.0)

        # ─────────────────────────────────────────────────────────
        # Test 4: Reset packet TX -> ERROR -> INIT return
        # ─────────────────────────────────────────────────────────
        separator("Test 4: Reset packet TX -> ERROR -> INIT return")
        print("► Transmitting flags bit0=1 (reset) packet...")
        reset_pkt = make_packet(reset=True)
        send(sock, addr, reset_pkt)
        print("✔ Reset packet TX completed.")
        print("  [ESP32 Serial] Check the following two logs in sequence:")
        print("    1. 'Reset command received — triggering INIT transition'")
        print("    2. '[Transition] ERROR -> INIT'")
        time.sleep(0.5)

        # ─────────────────────────────────────────────────────────
        separator("All tests completed")
        print("Check the pass criteria logs for each test in the serial monitor.")
        print(f"Packet size: {PACKET_BYTES} bytes / Port: {args.port}")


if __name__ == "__main__":
    main()
