@echo off
title zvey code - live maze map
python -m pip install --quiet --disable-pip-version-check pyserial matplotlib
python "%~dp0maze_route_viewer.py" %*
pause
