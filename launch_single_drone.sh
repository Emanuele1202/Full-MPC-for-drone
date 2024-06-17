#!/bin/bash

h2 init
genomixd &
optitrack-pocolibs -f &
rotorcraft-pocolibs -f &
pom-pocolibs -f &
maneuver-pocolibs -f &
nhfc-pocolibs -f &

