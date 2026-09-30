#!/usr/bin/env bash
# Build 1 app Trampoline (OSEK) tren Windows (Git Bash). CHI BUILD, khong nap.
#
#   scripts/trampoline_build.sh amr_trampoline/fpu_check [target]
#
# target mac dinh: cortex-m/armv7em/stm32f407/stm32f4discovery
# Yeu cau (xem docs/trampoline-setup.md):
#   - Trampoline clone tai ../trampoline (canh amr_ws), branch amr-f407
#   - goil da build: ../trampoline/goil/makefile-unix/goil.exe
#   - WinLibs GCC (goil can DLL runtime cua no) + GNU Tools for STM32 trong CubeIDE
set -euo pipefail

APP_DIR="${1:?Cach dung: $0 <thu muc app chua file .oil> [target]}"
TARGET="${2:-cortex-m/armv7em/stm32f407/stm32f4discovery}"

REPO="$(cd "$(dirname "$0")/.." && pwd)"
TPL="${TRAMPOLINE_DIR:-$(cd "$REPO/../trampoline" && pwd)}"
WINLIBS="${WINLIBS_BIN:-/c/winlibs/winlibs-x86_64-posix-seh-gcc-16.1.0-mingw-w64msvcrt-14.0.0-r4/mingw64/bin}"
ARM_BIN="${ARM_BIN:-$(ls -d /c/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin | head -1)}"

for p in "$TPL/goil/makefile-unix/goil.exe" "$WINLIBS/g++.exe" "$ARM_BIN/arm-none-eabi-gcc.exe"; do
  [ -f "$p" ] || { echo "Khong tim thay: $p" >&2; exit 1; }
done
export PATH="$ARM_BIN:$WINLIBS:$TPL/goil/makefile-unix:$PATH"

cd "$REPO/$APP_DIR"
OIL="$(ls *.oil | head -1)"
# goil can duong dan templates TUONG DOI (make.py goi lai goil tu thu muc app)
TEMPLATES="$(python -c "import os,sys;print(os.path.relpath(sys.argv[1]).replace(os.sep,'/'))" "$TPL/goil/templates")/"

goil --target="$TARGET" --templates="$TEMPLATES" "$OIL"
python make.py
