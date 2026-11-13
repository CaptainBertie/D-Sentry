import json,sys
def crc8(data):
    c=0
    for x in data:
        c^=x
        for _ in range(8):
            c=((c<<1)^0x07)&0xFF if c&0x80 else (c<<1)&0xFF
    return c
assert crc8(b"123456789")==0xF4, "crc check value"
print("CRC-8/0x07 check('123456789') = 0x%02X (expect 0xF4)" % crc8(b"123456789"))
for run in sys.argv[1:]:
    print("==", run)
    R=json.load(open(f"logs/{run}/at_results.json"))
    for r in R:
        b=bytes.fromhex(r["reply_hex"].replace(" ",""))
        extra=""
        if r["cmd"].startswith("AT+XBOXSTATE?") and b.startswith(b"+XBOXSTATE: 20|"):
            p=b[len(b"+XBOXSTATE: 20|"):][:20]
            tail=b[len(b"+XBOXSTATE: 20|")+20:]
            extra=f" payload={p.hex(' ')} len={len(p)} state={p[16]} batt=0x{p[17]:02X} seq={p[18]} crc=0x{p[19]:02X} calc=0x{crc8(p[:19]):02X} {'CRC OK' if crc8(p[:19])==p[19] else 'CRC BAD'} tail={tail!r}"
        print(f"t={r['t_sent']:7.3f} first_byte={r['first_byte_ms']}ms {r['cmd']!r:24} {r['len']:3d}B {r['reply_hex']}{extra}")
