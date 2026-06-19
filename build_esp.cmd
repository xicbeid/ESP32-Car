@echo off
REM ============================================
REM UART Project Build Script for ESP32-P4
REM Run this from Command Prompt (cmd.exe)
REM ============================================
set IDF_PATH=E:\Esp32\frameworks\esp-idf-v5.4
set IDF_TOOLS_PATH=E:\Esp32
call E:\Esp32\frameworks\esp-idf-v5.4\export.bat
cd /d E:\esp32-project\UART
idf.py %*
