#!/bin/sh
# Reproduce the QEMU runs. Emulation only; nothing touches hardware.
set -e
W="$(cd "$(dirname "$0")" && pwd)"
cd "$W"
SP="$(dirname "$W")"
PROJ="${PROJ:-$(cd "$W/../.." && pwd)}"   # the repo root; override with PROJ=...

# 1. Espressif QEMU 9.2.2 (esp-develop-9.2.2-20260417), macOS arm64 prebuilt
[ -x qemu/bin/qemu-system-xtensa ] || {
  curl -sLO https://github.com/espressif/qemu/releases/download/esp-develop-9.2.2-20260417/qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-aarch64-apple-darwin.tar.xz
  tar xf qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-aarch64-apple-darwin.tar.xz
}
qemu/bin/qemu-system-xtensa -M help | grep esp32s3

# 2. Images: byte-exact copies of the 4 MB S3-ALL.bin files, plus 16 MB copies padded with 0xFF
# A stock 4 MB image: build it from the vendor tag (git worktree add /tmp/stock vendor/uno-r4-wifi-usb-bridge; ./build.sh there)
# or point STOCK_IMG at Arduino's release image padded to 4 MB. Default: the copy kept by the session that wrote this.
cp "${STOCK_IMG:-$SP/stock-build/S3-ALL.bin}" stock-flash.bin
cp "$PROJ/firmware/UNOR4USBBridge/build/esp32-patched.esp32.arduino_unor4wifi_usb_bridge/S3-ALL.bin" fork-flash.bin
for i in stock fork; do
  cp $i-flash.bin $i-flash-16M.bin
  python3 -c "open('$i-flash-16M.bin','ab').write(b'\xff'*(12*1024*1024))"
  dd if=/dev/zero bs=1024 count=1 of=efuse-$i.bin 2>/dev/null
done

# 3. 4 MB image as-is: boot loop (QEMU picks the gd25q32 model; esp_flash_init_main -> 0x108)
#    qemu/bin/qemu-system-xtensa -nographic -M esp32s3 -drive file=fork-flash.bin,if=mtd,format=raw \
#      -drive file=efuse-fork.bin,if=none,format=raw,id=efuse \
#      -global driver=nvram.esp32s3.efuse,property=drive,value=efuse -serial file:uart0.log -serial null

# 4. Harness runs (UART0 and UART1 on TCP, QMP; t=0 at "cont")
mkdir -p run
for r in stock-B fork-B; do i=${r%-B}; cp $i-flash-16M.bin run/$r-flash.bin; cp efuse-$i.bin run/$r-efuse.bin; done
python3 run_sim.py stock-B run/stock-B-flash.bin run/stock-B-efuse.bin --cmds stock --idle 30 --probe-ms 20
python3 run_sim.py fork-B  run/fork-B-flash.bin  run/fork-B-efuse.bin  --cmds fork  --idle 30 --probe-ms 20
python3 analyze.py fork-B stock-B
