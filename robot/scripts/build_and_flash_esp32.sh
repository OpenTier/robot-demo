#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

source ~/opentier-robot-demo/scripts/setup_esp32.sh

cd ~/opentier-robot-demo/robot
platformio run --target upload
