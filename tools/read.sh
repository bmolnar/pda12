#!/bin/sh

dd if=./dev bs=65536 count=1 | hexdump -x
