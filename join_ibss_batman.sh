#!/bin/bash

# 传入参数
IFACE=$1             # 无线设备名
IPADDR=$2            # BATMAN IP，例如 192.168.199.1

# 固定配置
ESSID="my-ibss-network"
FREQ=2412            # Channel 1
BAND="HT20"

if [ -z "$IFACE" ] || [ -z "$IPADDR" ]; then
    echo "用法: $0 <网卡设备名> <bat0 IP地址>"
    echo "例如: $0 wlx90de806c95e9 192.168.199.1"
    exit 1
fi

echo ">>> 停用 NetworkManager 对 $IFACE 的管理"
nmcli dev set $IFACE managed no

echo ">>> 关闭接口 $IFACE"
ip link set $IFACE down

echo ">>> 设置 IBSS 模式"
iw $IFACE set type ibss

echo ">>> 启用 batman-adv 模块"
modprobe batman-adv

echo ">>> 添加 $IFACE 到 batman-adv"
batctl if add $IFACE

echo ">>> 启用接口 $IFACE 和 bat0"
ip link set $IFACE up

echo ">>> 加入 IBSS 网络 $ESSID"
iw $IFACE ibss join "$ESSID" $FREQ $BAND #fixed-freq 02:11:22:33:44:55

ip link set bat0 up

echo ">>> 设置 bat0 IP 地址 $IPADDR"
ip addr add $IPADDR/24 dev bat0

echo ">>> 设置完成！当前 bat0 状态："
ip addr show bat0

echo ">>> 加入wg0加密网络"
sudo wg-quick up wg0

echo ">>> 加密网络配置："
sudo wg show

