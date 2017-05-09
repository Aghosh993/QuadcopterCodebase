#!/bin/bash
ip link set up can0 type can bitrate 500000
ifconfig can0 up
