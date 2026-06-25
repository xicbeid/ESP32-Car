"""Build wrapper for ESP32-P4 UART WiFi Car (IDF v5.5.4)."""
import os, sys, subprocess

IDF_PATH          = r'F:\Espressif\frameworks\v5.5.4\esp-idf'
IDF_TOOLS_PATH    = r'F:\Espressif'
IDF_PYTHON        = r'F:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe'
PROJ_DIR          = r'F:\project_ESP32_p4\UART'
TOOLCHAIN_RISCV   = r'F:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin'
TOOLCHAIN_XTENSA  = r'F:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20240906\xtensa-esp-elf\bin'
CMAKE_BIN         = r'F:\Espressif\tools\cmake\3.30.2\bin'
NINJA_BIN         = r'F:\Espressif\tools\ninja\1.12.1'

os.environ.pop('MSYSTEM', None)
os.environ['IDF_PATH']          = IDF_PATH
os.environ['IDF_PYTHON_ENV_PATH'] = r'F:\Espressif\python_env\idf5.5_py3.11_env'
os.environ['IDF_TOOLS_PATH']    = IDF_TOOLS_PATH

for p in [CMAKE_BIN, NINJA_BIN, TOOLCHAIN_RISCV, TOOLCHAIN_XTENSA]:
    os.environ['PATH'] = p + os.pathsep + os.environ.get('PATH', '')

idf_py = os.path.join(IDF_PATH, 'tools', 'idf.py')
args = sys.argv[1:] if len(sys.argv) > 1 else ['build']
sys.exit(subprocess.call([IDF_PYTHON, idf_py] + args, cwd=PROJ_DIR))
