#!/usr/bin/bash

#Configs here

#Zig sourcing
nightly_checker=~/.config/zig/nightly_checker.sh
ziglocfile=~/.config/zig/zigloc

source $nightly_checker
source $ziglocfile

echo "Zig initialized in the workspace"
