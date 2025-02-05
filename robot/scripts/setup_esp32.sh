#!/bin/bash
# Set up ESP-IDF environment variables
export IDF_PATH=/opt/esp-idf
export PATH="$IDF_PATH/tools:$PATH"

# Source the ESP-IDF export script if it exists
if [ -f /opt/esp-idf/export.sh ]; then
    source /opt/esp-idf/export.sh
fi
