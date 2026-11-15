import sys, statistics as st, collections
from rlogparse import parse
D,_=parse(sys.argv[1])
for side in ('Left','Right'):
    S=D.get(f'RealOutputs/Vision/{side}/Status',[])
    print(side,'status changes:')
    for t,v in S: print('   %.1f %s'%(t,v))
    ids=D.get(f'Vision/{side}/TagIds',[]); print('  tagid samples',len(ids), 'fps last', D.get(f'Vision/{side}/Fps',[])[-1:])
    c=collections.Counter(i for t,v in ids for i in v); print('  ids',dict(c))
ts=[t for k,v in D.items() for t,_ in v]; print('span',min(ts),max(ts))
