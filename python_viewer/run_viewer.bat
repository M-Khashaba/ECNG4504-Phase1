@echo off
title Maze robot live map
python -m pip install --quiet --disable-pip-version-check pyserial matplotlib
python "%~dp0maze_viewer.py" %*
pause