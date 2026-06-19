@echo off
set IDF_PATH=E:\Esp32\frameworks\esp-idf-v5.4
set IDF_TOOLS_PATH=E:\Esp32
call E:\Esp32\frameworks\esp-idf-v5.4\export.bat
cd /d E:\esp32-project\UART
idf.py --version
idf.py set-target esp32p4
idf.py build
