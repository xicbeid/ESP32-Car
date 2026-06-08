@echo off
set IDF_PATH=F:\Espressif\frameworks\esp-idf-v5.4
set IDF_TOOLS_PATH=F:\Espressif
call F:\Espressif\frameworks\esp-idf-v5.4\export.bat
cd /d F:\project_ESP32_p4\UART
idf.py --version
idf.py set-target esp32p4
idf.py build
