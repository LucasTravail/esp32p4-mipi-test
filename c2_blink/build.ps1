# Builds the ESP8684 test firmware and copies the MERGED flash image (bootloader + partition table + app, flashed at 0x0) to
# ../main/c2_blink.bin (embedded in the P4 firmware). Note: build\c2_blink.bin is the app alone, it must not be flashed at 0x0.
$env:IDF_PATH='C:\esp\v5.5.5\esp-idf'; $env:IDF_TOOLS_PATH='C:\Espressif'; $env:IDF_PYTHON_ENV_PATH='C:\Espressif\tools\python\v5.5.5\venv'
Set-Location $PSScriptRoot
# the toolchain sometimes ICEs: just rerun, the build resumes
cmd /d /c "call C:\esp\v5.5.5\esp-idf\export.bat >nul 2>&1 && idf.py set-target esp32c2 && idf.py -DCCACHE_ENABLE=0 build -- -j2 && idf.py merge-bin -o c2_blink_merged.bin"
Copy-Item build\c2_blink_merged.bin ..\main\c2_blink.bin -Force
