#!/bin/bash
# Run through setup.py unlock. Private key stays encrypted on disk.
set -eu
export SSH_AUTH_SOCK="$HOME/.ssh/spirob-camera-agent.sock"
spirob_status=0
ssh-add -l >/dev/null 2>&1 || spirob_status=$?
if [ "$spirob_status" -eq 2 ]; then
    if [ -S "$SSH_AUTH_SOCK" ]; then rm -- "$SSH_AUTH_SOCK"; fi
    eval "$(ssh-agent -a "$SSH_AUTH_SOCK" -s)"
fi
ssh-add -t 8h "$1"
printf '\nCamera key is available to the WSL coordinator for up to eight hours.\n'
