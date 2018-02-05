#!/bin/bash

echo "Starting shutdown button"
/home/robot/shutdown/bin/shutdown &

echo "Starting statuslight"
/home/robot/statuslight/bin/statuslight &

echo "Done!"
