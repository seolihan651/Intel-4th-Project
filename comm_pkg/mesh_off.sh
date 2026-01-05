#!/usr/bin/env bash
set -euo pipefail

IFACE="wlan0"

echo "[mesh_off] start"

# bat0 내리고 인터페이스 제거
sudo ip addr flush dev bat0 2>/dev/null || true
sudo ip link set bat0 down 2>/dev/null || true
sudo batctl if del "$IFACE" 2>/dev/null || true

# IBSS 정리
sudo iw dev "$IFACE" ibss leave 2>/dev/null || true

# wlan0 리셋
sudo ip addr flush dev "$IFACE" 2>/dev/null || true
sudo ip link set "$IFACE" down 2>/dev/null || true
sudo iw dev "$IFACE" set type managed 2>/dev/null || true
sudo ip link set "$IFACE" up 2>/dev/null || true

# 네트워크 서비스 다시 켜기(환경에 따라 하나만 살아있어도 됨)
sudo systemctl start NetworkManager 2>/dev/null || true
sudo systemctl start wpa_supplicant 2>/dev/null || true
sudo systemctl start dhcpcd 2>/dev/null || true

echo "[mesh_off] done"
