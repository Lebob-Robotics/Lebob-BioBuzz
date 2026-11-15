"""Minimal PsiKit/AdvantageKit .rlog reader: parse(path) -> {key: [(t, value)]}, types."""
import struct

def _decode(typ, b):
    try:
        if typ == 'boolean': return b[0] != 0
        if typ == 'int64': return struct.unpack('>q', b)[0]
        if typ == 'float': return struct.unpack('>f', b)[0]
        if typ == 'double': return struct.unpack('>d', b)[0]
        if typ == 'string': return b.decode('utf-8', 'replace')
        if typ == 'boolean[]': return [x != 0 for x in b]
        if typ == 'int64[]': return list(struct.unpack('>%dq' % (len(b)//8), b))
        if typ == 'float[]': return list(struct.unpack('>%df' % (len(b)//4), b))
        if typ == 'double[]': return list(struct.unpack('>%dd' % (len(b)//8), b))
        if typ.startswith('struct:') and len(b) % 8 == 0:
            return list(struct.unpack('<%dd' % (len(b)//8), b))
    except Exception:
        pass
    return b

def parse(path):
    data = open(path, 'rb').read()
    i, n = 1, len(data)
    names, types, series = {}, {}, {}
    t = 0.0
    while i < n:
        kind = data[i]; i += 1
        if kind == 0:
            if i + 8 > n: break
            t = struct.unpack('>d', data[i:i+8])[0]; i += 8
        elif kind == 1:
            if i + 4 > n: break
            kid, ln = struct.unpack('>hh', data[i:i+4]); i += 4
            name = data[i:i+ln].decode('utf-8', 'replace'); i += ln
            (tl,) = struct.unpack('>h', data[i:i+2]); i += 2
            typ = data[i:i+tl].decode('utf-8', 'replace'); i += tl
            names[kid] = name; types[name] = typ
        elif kind == 2:
            if i + 4 > n: break
            kid, ln = struct.unpack('>hH', data[i:i+4]); i += 4
            if i + ln > n: break  # Pulled while still being written
            payload = data[i:i+ln]; i += ln
            name = names.get(kid)
            if name is not None:
                series.setdefault(name, []).append((t, _decode(types[name], payload)))
        else:
            break
    return series, types
