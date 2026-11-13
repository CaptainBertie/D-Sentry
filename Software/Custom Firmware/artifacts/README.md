# Prebuilt images

`xbox-0.6.99/S3-BOOT-APP.bin` is the forked bridge firmware exactly as `firmware/build.sh`
produced it on 2026-10-11 (commit recorded in the git log for this file). Flash it at 0x0 with
`tools/r4flash.py flash artifacts/xbox-0.6.99/S3-BOOT-APP.bin --backup <your backup>`.
`UNOR4USBBridge.ino.bin` is the app alone (for `image-info` or an app-only write at 0x50000).
`SHA256SUMS` lets you confirm a rebuild reproduces the same bytes (a rebuild from a different
checkout path will differ slightly: the build embeds the path string).

Arduino's official stock image (`UNOR4-WIFI-S3-0.6.0.bin`, sha256
`e4a0e6f9451188b5714c2ec53a6552d2ad59b59f468cd47b695bd19ba752fd3c`) is not stored here;
`tools/r4flash.py restore --official` downloads and verifies it, and a copy sits in
`tools/cache/` on the build machine.
