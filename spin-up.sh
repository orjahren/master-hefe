#!/usr/bin/zsh

# Hefe
gnome-terminal --working-directory="$HEFE_ROOT" -- zsh -c "make; exec zsh"

# Carla
gnome-terminal --working-directory="$CARLA_ROOT" 

# Scenario Runner, running scenarios
gnome-terminal --working-directory="$SCENARIO_RUNNER_ROOT"

# Scenario Runner, running ego vehicle
gnome-terminal --working-directory="$SCENARIO_RUNNER_ROOT"
