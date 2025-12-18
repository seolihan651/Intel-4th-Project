#!/usr/bin/env bash
set -e

IF=wlan0
SSID="turtle-mesh"
FREQ="2437"
BAT0_IP="10.50.0.11/24"

# 방해하는 네트워크 관리 서비스 잠깐 내려서 wlan0가 managed로 돌아가는 걸 방지
systemctl stop NetworkManager 2>/dev/null || true
systemctl stop wpa_supplicant 2>/dev/null || true
systemctl stop dhcpcd 2>/dev/null || true
pkill wpa_supplicant 2>/dev/null || true

rfkill unblock wifi || true

# 1) IBSS(Ad-hoc) 조인
ip link set $IF down || true
iw dev $IF set type ibss
ip link set $IF up
iw dev $IF ibss leave 2>/dev/null || true
iw dev $IF ibss join "$SSID" $FREQ fixed-freq

# 절전모드 끄기(지연 튐/끊김 완화)
iw dev $IF set power_save off 2>/dev/null || true

# 2) batman-adv + bat0 구성
modprobe batman-adv
batctl if del $IF 2>/dev/null || true

ip addr flush dev $IF   # wlan0는 링크 전용(IP 없음)
batctl if add $IF

ip link set up dev bat0
ip link set dev bat0 mtu 1460

ip addr flush dev bat0
ip addr add $BAT0_IP dev bat0
