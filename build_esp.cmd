@echo off
REM ============================================
REM UART Project Build Script for ESP32-P4
REM Run this from Command Prompt (cmd.exe)
REM ============================================
set IDF_PATH=F:\Espressif\frameworks\esp-idf-v5.4
set IDF_TOOLS_PATH=F:\Espressif
call F:\Espressif\frameworks\esp-idf-v5.4\export.bat
cd /d F:\project_ESP32_p4\UART
idf.py %*
