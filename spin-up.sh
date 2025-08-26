#!/usr/bin/bash

# Hefe
gnome-terminal --working-directory="$HEFE_ROOT" -- make

# Carla
gnome-terminal --working-directory="$CARLA_ROOT" 

# Scenario Runner
gnome-terminal --working-directory="$SCENARIO_RUNNER_ROOT"
