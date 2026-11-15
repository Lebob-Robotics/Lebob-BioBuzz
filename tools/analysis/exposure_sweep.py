import sys, re, statistics as st, collections, bisect
from rlogparse import parse
D,_=parse(sys.argv[1])
def ff(series):
    ts=[t for t,_ in series]; vs=[v for _,v in series]
    def at(t):
        k=bisect.bisect_right(ts,t)-1
        return vs[k] if k>=0 else None
    return at
for side in ('Left','Right'):
    p=f'Vision/{side}/'
    S=D.get('RealOutputs/'+p+'Status',[])
    chg=[]
    for t,v in S:
        m=re.search(r'exposure (\d+) ms, gain (\d+)',v)
        if m: chg.append((t,(int(m.group(1)),int(m.group(2)))))
    print(f'== {side}')
    fr=D.get(p+'FrameNanos',[])
    get={k:ff(D.get(p+k,[])) for k in ('TagIds','XIn','YIn','ZIn','DecisionMargin')}
    for j,(t0,setting) in enumerate(chg):
        t1=chg[j+1][0] if j+1<len(chg) else 1e18
        a=t0+1.5
        frames=[t for t,v in fr if a<=t<t1]
        per=collections.defaultdict(lambda:{'n':0,'m':[],'p':[]})
        for t in frames:
            ids=get['TagIds'](t) or []
            X,Y,Z,M=(get[k](t) for k in ('XIn','YIn','ZIn','DecisionMargin'))
            for k,tid in enumerate(ids):
                q=per[tid]; q['n']+=1
                try: q['m'].append(M[k]); q['p'].append((X[k],Y[k],Z[k]))
                except Exception: pass
        line=f'  exp {setting[0]:2d} gain {setting[1]:3d} frames {len(frames):3d} |'
        for tid in sorted(per):
            q=per[tid]
            if len(q['p'])<2: continue
            mx=[st.mean(c) for c in zip(*q['p'])]
            r=sum(c*c for c in mx)**0.5
            jit=sum(st.pstdev(c)**2 for c in zip(*q['p']))**0.5
            line+=f' {tid}:{q["n"]/len(frames):.0%} m{st.mean(q["m"]):.0f} r{r*0.0254:.3f} j{jit*25.4:.1f}mm'
        print(line)
