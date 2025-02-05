#!/bin/bash
set -e

# -- Configuration --
REMOTE_USER="${REMOTE_USER:-demo}"
REMOTE_HOST="${REMOTE_HOST:-u-opentier-pi4.local}"

# Ensure SSH key-based authentication is set up to avoid password prompts
ensure_ssh_is_setup() {
    if ! ssh -o BatchMode=yes "${REMOTE_USER}@${REMOTE_HOST}" 'exit' 2>/dev/null; then
        echo "SSH key-based authentication is not set up. Setting it up now..."
        ssh-copy-id "${REMOTE_USER}@${REMOTE_HOST}"
    fi
}

get_ip() {
    echo "Getting the IP address of the Raspberry Pi..."
    IP_ADDRESS=$(ssh "${REMOTE_USER}@${REMOTE_HOST}" "hostname -I | cut -d' ' -f1")
    echo $IP_ADDRESS
}

ensure_ssh_is_setup
get_ip