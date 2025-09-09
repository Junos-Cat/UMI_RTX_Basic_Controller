#!/bin/bash
# Diagnostic script for armlib / RTX socket

# These are the defaults from rtx.h / comm.h
RTXD_DIR="/home/Stage/LAB42_RTX_control/umi-rtx"   # adjust if different
RTXD_SOCKET="ports/rtx-socket"
RTXD_PORTINFO="ports/rtx-portinfo"

echo "=== RTX Diagnostic ==="

# Check directory
echo "RTXD_DIR: $RTXD_DIR"
if [ -d "$RTXD_DIR" ]; then
    echo "Directory exists"
else
    echo "Directory MISSING"
fi

# Check socket file
SOCKET_PATH="$RTXD_DIR/$RTXD_SOCKET"
echo "RTXD_SOCKET: $SOCKET_PATH"
if [ -e "$SOCKET_PATH" ]; then
    echo "Socket exists"
    ls -l "$SOCKET_PATH"
else
    echo "Socket MISSING"
fi

# Check portinfo file
PORTINFO_PATH="$RTXD_DIR/$RTXD_PORTINFO"
echo "RTXD_PORTINFO: $PORTINFO_PATH"
if [ -e "$PORTINFO_PATH" ]; then
    echo "Portinfo exists"
    ls -l "$PORTINFO_PATH"
else
    echo "Portinfo MISSING"
fi
