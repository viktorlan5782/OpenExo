@echo off
title VLE Exo GUI Launcher
cd /d "E:\6_Exo_Coding\VLExo_202604\VLE_1\Python_GUI"
C:\Users\viktor\AppData\Local\Programs\Python\Python39\python.exe GUI.py
if errorlevel 1 (
    echo.
    echo [ERROR] Program exited with error. Press any key to close...
    pause >nul
)