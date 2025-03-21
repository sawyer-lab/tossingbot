#!/bin/env bash
docker stop robo2025; ./build.sh && ./run.sh && ./exec.sh
