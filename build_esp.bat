@echo off
set MSYSTEM=
set IDF_PATH=F:\Espressif\frameworks\v5.5.4\esp-idf
set IDF_TOOLS_PATH=F:\Espressif
call F:\Espressif\frameworks\v5.5.4\esp-idf\export.bat
cd /d F:\project_ESP32_p4\UART
idf.py --version
idf.py set-target esp32p4
idf.py build
