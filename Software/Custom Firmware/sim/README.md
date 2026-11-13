# Simulation (added 2026-10-12)

There is no emulator for the whole UNO R4 WiFi, so the two chips were simulated separately.
Neither run touched a board, and neither needs an account.

| Folder | Simulates | Proves | Cannot prove |
|---|---|---|---|
| `qemu/` | The ESP32-S3 running the real forked `S3-ALL.bin` in Espressif's QEMU | Boot path, partition layout, `setup()` survives, every pre-BEGIN `AT+XBOX*` reply byte-exact, the HCI guard | Anything Bluetooth (QEMU has no BT; `AT+XBOXBEGIN` hits an IDF assert that stock `AT+HCIBEGIN` hits too), NVS, USB, WiFi |
| `renode/` | The RA4M1 running the real unmodified sketch ELF in Renode 1.17.0, with a Python stand-in for the ESP32-S3 on the AT link | The whole RA4M1 software path: Arduino core, FSP UART, WiFiS3 `Modem`, `XboxControllerR4`; `begin()`, 50 Hz polling (976 polls in 20 s, none lost), decoding of sticks, buttons, battery, seq wrap, the error-1 (stock) and error-4 (silent) paths, StickToLED brightness | Real UART timing, the ESP32 firmware itself, BLE, the LED matrix, ADC |

The two could be joined (Renode's SCI1 on TCP port 3456 to QEMU's UART1) to run the real
firmware against the real library, but with no Bluetooth in QEMU the controller would never
connect, so the Python stand-in is the better test of the RA4M1 side.

Wokwi was ruled out: it has no UNO R4 or RA4M1 board. Renesas's e2 studio simulator covers RX
and RL78 only.
