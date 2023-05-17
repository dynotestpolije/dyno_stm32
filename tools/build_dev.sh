#!/usr/bin/env bash
source $(dirname $0)/incl.sh

if [[ -d "./build/debug" ]]; then
    make check 
    echo "building..."
    make -j build DEBUG=1  # >/dev/null 
    echo "done"
else
    [[ -f "./compile_commands.json" ]] && rm -rf ./compile_commands.json;
    make check
    echo "building..."
    bear -- make -j build DEBUG=1 # >/dev/null 
    echo "done"
fi

