#!/usr/bin/env bash
pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f ruby
pkill -9 -f gzserver; pkill -9 -f gzclient
pkill -9 -f parameter_bridge; pkill -9 -f arms_
pkill -9 -f socat; pkill -9 -f micro; pkill -9 -f sitl_bridge
pkill -9 -f balloon_referee
sleep 3
ros2 daemon stop; sleep 1; ros2 daemon start
echo "✅ 전체 종료 완료"
