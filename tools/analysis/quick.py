import sys
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from rlogparse import parse
d,_=parse(sys.argv[1])
P=[v for t,v in d.get('RealOutputs/Odometry/Pose',[])]
ph=d.get('RealOutputs/FieldTest/Phase',[])
print('phase', ph[-1][1] if ph else None, 'trial', d.get('RealOutputs/FieldTest/Trial',[(0,None)])[-1][1], 'lap', d.get('RealOutputs/FieldTest/Lap',[(0,None)])[-1][1], 'fin', d.get('RealOutputs/FieldTest/FinishReason',[(0,'')])[-1][1])
if P: print('max|x|', round(max(abs(v[0]) for v in P),2), 'max|y|', round(max(abs(v[1]) for v in P),2), 'now', [round(x,2) for x in P[-1]])
import collections
print(collections.Counter(v for t,v in d.get('RealOutputs/Vision/Correction/Reason',[])).most_common(5))
