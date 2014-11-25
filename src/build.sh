#!/bin/sh

make SUBDIRS=${PWD} -C /lib/modules/$(uname -r)/build

