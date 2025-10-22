#!/usr/bin/env bash

NO_CACHE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--no-cache]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--no-cache]"
            exit 1
            ;;
    esac
done

docker build \
    $NO_CACHE \
    --build-arg HOST_HOSTNAME=$(hostname) \
    --build-arg HOST_IP=$(hostname -I | cut -d ' ' -f 1) \
    -t robo2025-workspace workspace

