import math

def tofloat(s):
    s=s.strip()
    if not s or s in ("NDM","NaN") or "*" in s: return float("nan")
    try: return float(s)
    except: return float("nan")

with open("datcom.out","r",errors="ignore") as f:
    lines=f.readlines()

VALID_ALPHA={0.0,4.0,8.0,12.0,16.0,20.0}
MACH=[0.8,1.2,1.6,2.0,3.0]
mach_blocks=[]
current=[]
in_block=False

for i,line in enumerate(lines):
    is_header="ALPHA" in line and "CD" in line and "CL" in line and "CN" in line
    if is_header:
        if current: mach_blocks.append(current[:6])
        current=[]
        in_block=True
        continue
    if not in_block: continue
    if "NDM PRINTED" in line: in_block=False; continue
    if len(current)==6: in_block=False; continue
    if len(line)<10: continue
    alpha=tofloat(line[0:7])
    if math.isnan(alpha) or alpha not in VALID_ALPHA: continue
    current.append([alpha,tofloat(line[7:16]),tofloat(line[16:25]),tofloat(line[25:34]),tofloat(line[34:43]),tofloat(line[43:52])])

if current: mach_blocks.append(current[:6])

print(f"Blocks: {len(mach_blocks)}")
for i,blk in enumerate(mach_blocks):
    print(f"\nMach={MACH[i] if i<5 else i}")
    print("  alpha    CD       CL       CM       CN       CA")
    for r in blk:
        def fmt(v): return f"{v:7.3f}" if not math.isnan(v) else "    nan"
        print(f"  {r[0]:5.1f}  {'  '.join(fmt(v) for v in r[1:])}")
