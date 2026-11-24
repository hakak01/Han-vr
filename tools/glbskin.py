"""Write a skinned GLB: one glTF skin shared by all meshes; joint nodes have identity rest transforms,
so a joint's runtime matrix = the rig's bone world matrix (rest -> pose) and inverseBindMatrices = identity.
Extra per-vertex float attributes (names starting with '_') are written as-is."""
import json, struct
import numpy as np
from glbio import vertex_normals


def write(path, objects, joints, joint_extras=None, extras=None):
    """objects: list of (name, [prim]) with prim = dict(v, f, mat, J (N x 4 uint8), W (N x 4 float), attrs={name: array})."""
    blob = bytearray(); views = []; accs = []; mats = []; meshes = []; nodes = []

    def add(arr, typ, comp, target=34962, minmax=False):
        arr = np.ascontiguousarray(arr)
        while len(blob) % 4:
            blob.append(0)
        views.append(dict(buffer=0, byteOffset=len(blob), byteLength=arr.nbytes, **({'target': target} if target else {})))
        blob.extend(arr.tobytes())
        n = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}[typ]
        a = dict(bufferView=len(views) - 1, componentType=comp, count=arr.size // n, type=typ)
        if minmax:
            a['min'] = arr.reshape(-1, n).min(0).tolist(); a['max'] = arr.reshape(-1, n).max(0).tolist()
        accs.append(a)
        return len(accs) - 1

    for i, jn in enumerate(joints):
        nd = dict(name=jn)
        if joint_extras and jn in joint_extras:
            nd['extras'] = joint_extras[jn]
        nodes.append(nd)
    ibm = add(np.tile(np.eye(4, dtype=np.float32).reshape(-1), len(joints)), 'MAT4', 5126, target=None)
    for name, prims in objects:
        ps = []
        for p in prims:
            if p['mat'] not in mats:
                mats.append(p['mat'])
            v = np.asarray(p['v'], np.float32); f = np.asarray(p['f'], np.uint32)
            at = dict(POSITION=add(v, 'VEC3', 5126, minmax=True),
                      NORMAL=add(vertex_normals(v.astype(np.float64), f.astype(np.int64)).astype(np.float32), 'VEC3', 5126),
                      JOINTS_0=add(np.asarray(p['J'], np.uint8), 'VEC4', 5121),
                      WEIGHTS_0=add(np.asarray(p['W'], np.float32), 'VEC4', 5126))
            for k, a in (p.get('attrs') or {}).items():
                a = np.asarray(a, np.float32)
                at[k] = add(a, 'SCALAR' if a.ndim == 1 else 'VEC4', 5126)
            ps.append(dict(attributes=at, indices=add(f.reshape(-1), 'SCALAR', 5125, 34963),
                           material=mats.index(p['mat'])))
        meshes.append(dict(name=name, primitives=ps))
        nodes.append(dict(name=name, mesh=len(meshes) - 1, skin=0))
    j = dict(asset=dict(version='2.0', generator='Han-vr bind_tissues.py'), scene=0,
             scenes=[dict(nodes=list(range(len(nodes))))], nodes=nodes, meshes=meshes,
             skins=[dict(name='Hand rig', joints=list(range(len(joints))), inverseBindMatrices=ibm)],
             materials=[dict(name=m, pbrMetallicRoughness=dict(metallicFactor=0, roughnessFactor=0.7)) for m in mats],
             accessors=accs, bufferViews=views, buffers=[dict(byteLength=len(blob))])
    if extras:
        j['extras'] = extras
    js = json.dumps(j, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    while len(blob) % 4:
        blob.append(0)
    with open(path, 'wb') as fh:
        fh.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob)))
        fh.write(struct.pack('<II', len(js), 0x4E4F534A)); fh.write(js)
        fh.write(struct.pack('<II', len(blob), 0x004E4942)); fh.write(bytes(blob))


def read(path):
    """-> (json, {node name: [dict(v, f, mat, attrs)]}) for files written by write() (uncompressed)."""
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[12:16])[0]
    j = json.loads(b[20:20 + n])
    binp = 20 + n + 8
    CT = {5126: np.float32, 5125: np.uint32, 5121: np.uint8}
    NC = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
    def acc(i):
        a = j['accessors'][i]; bv = j['bufferViews'][a['bufferView']]
        k = NC[a['type']]
        x = np.frombuffer(b, CT[a['componentType']], a['count'] * k, binp + bv.get('byteOffset', 0))
        return x.reshape(-1, k) if k > 1 else x
    out = {}
    for nd in j['nodes']:
        if 'mesh' not in nd:
            continue
        ps = []
        for p in j['meshes'][nd['mesh']]['primitives']:
            at = {k: acc(i) for k, i in p['attributes'].items()}
            ps.append(dict(v=at.pop('POSITION').astype(np.float64), f=acc(p['indices']).reshape(-1, 3).astype(np.int64),
                           mat=j['materials'][p['material']]['name'], attrs=at))
        out[nd['name']] = ps
    return j, out
