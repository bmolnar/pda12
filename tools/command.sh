#!/bin/sh

if [ -z "$1" ]; then
    echo "Enter a Command"
    exit 1
fi

dd if=/dev/zero of=./dev count=1 bs=$1
dmesg | tail -20
