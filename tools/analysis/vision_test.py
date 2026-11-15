import sys, collections, statistics as st, re
from rlogparse import parse
D,_=parse(sys.argv[1])
g=lambda k: D.get(k,[])
print('metadata', [v for t,v in g('RealMetadata/OpMode')][:1] if g('RealMetadata/OpMode') else '')
for s in ('Left','Right'):
    st_=[v for t,v in g(f'RealOutputs/Vision/{s}/Status')]
    ids=collections.Counter(); 
    for t,v in g(f'Vision/{s}/TagIds'): ids.update(v)
    fr=len(g(f'Vision/{s}/FrameNanos'))
    print(s,'status',st_[-1] if st_ else None,'| frames',fr,'| tag-list changes',dict(ids))
acc=g('RealOutputs/Vision/Correction/Accepted'); rs=collections.Counter(v for t,v in g('RealOutputs/Vision/Correction/Reason'))
print('reasons (on change):',dict(rs))
print('poseValid',[v for t,v in g('RealOutputs/Vision/Correction/PoseValid')][-1:], 'headingValid',[v for t,v in g('RealOutputs/Vision/Correction/HeadingValid')][-1:])
P=g('RealOutputs/Odometry/Pose')
print('pose first',P[0][1] if P else None,'last',P[-1][1] if P else None)
if P:
    t0=P[0][0]
    for t,v in P[::max(1,len(P)//15)]: print(f'  t{t-t0:6.1f} x{v[0]:+.3f} y{v[1]:+.3f} h{v[2]*57.2958:+7.2f}')
for k in ('DxM','DyM','DHeadingDeg','SpreadM','TagsUsed','Gain','HeadingPairs'):
    vals=[v for t,v in g('RealOutputs/Vision/Correction/'+k)]
    if vals and isinstance(vals[0],(int,float)):
        print(k,'n',len(vals),'median',round(st.median(vals),4),'min',round(min(vals),4),'max',round(max(vals),4))
ud=g('RealOutputs/Vision/Correction/UsedIds'); dx=dict(g('RealOutputs/Vision/Correction/UsedDxM')); dy=dict(g('RealOutputs/Vision/Correction/UsedDyM'))
per=collections.defaultdict(list)
for t,v in ud:
    if t in dx and t in dy:
        for i,tid in enumerate(v):
            try: per[tid].append((dx[t][i],dy[t][i]))
            except: pass
for tid in sorted(per):
    a=per[tid]; print(f'tag {tid}: n{len(a)} dx med {st.median(x for x,y in a):+.3f} dy med {st.median(y for x,y in a):+.3f}')
