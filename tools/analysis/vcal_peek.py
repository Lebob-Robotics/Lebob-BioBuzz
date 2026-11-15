import sys, collections
from rlogparse import parse
D,_=parse(sys.argv[1])
for side in ('Left','Right'):
    ids=D.get(f'RealOutputs/Vision/{side}/TagIds') or D.get(f'Vision/{side}/TagIds') or []
    keys=[k for k in D if f'/{side}/' in k][:0]
    c=collections.Counter()
    for t,v in ids:
        for i in v: c[i]+=1
    print(side,'samples',len(ids),'tag counts',dict(c))
print([k for k in D if 'Left' in k][:40])
