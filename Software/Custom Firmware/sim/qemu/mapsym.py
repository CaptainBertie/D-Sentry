import re,sys,bisect
mp=sys.argv[1]; addrs=[int(a,16) for a in sys.argv[2:]]
syms=[]
lines=open(mp,errors='replace').read().splitlines()
pend=None
for i,l in enumerate(lines):
    m=re.match(r'^ (\.\S+)\s*$',l)
    if m: pend=m.group(1); continue
    m=re.match(r'^ (\.\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)',l)
    if m: syms.append((int(m.group(2),16),int(m.group(3),16),m.group(1),m.group(4))); pend=None; continue
    m=re.match(r'^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)',l)
    if m and pend: syms.append((int(m.group(1),16),int(m.group(2),16),pend,m.group(3))); pend=None; continue
    m=re.match(r'^\s+0x([0-9a-f]+)\s+([A-Za-z_]\w*)\s*$',l)
    if m: syms.append((int(m.group(1),16),0,m.group(2),''))
syms.sort()
for a in addrs:
    best=[s for s in syms if s[0]<=a and (s[1]==0 or a<s[0]+s[1]) ]
    sec=[s for s in best if s[1]>0]
    nm=[s for s in best if s[1]==0]
    s=sec[-1] if sec else None
    print(hex(a), s[2] if s else '?', '+'+hex(a-s[0]) if s else '', (s[3].split('/')[-1] if s else ''))
