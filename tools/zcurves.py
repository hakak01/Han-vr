"""Phase E2 helper: centrelines of Z-Anatomy CURVE objects (nerves, vessels) -> data/raw/curves.json.
Run: python3 tools/zcurves.py   (needs bpy + data/raw/zblend/Z-Anatomy/Startup.blend; ~1 min)
Output per structure name (same names as hand.glb): {"lines": [[[x,y,z], ...], ...], "mean": [x,y,z]}
in glTF axes (Y up), metres, NOT yet shifted to the capitate origin ("mean" = mean of the bevelled mesh
vertices, used by bind_tissues.py to find the shift)."""
import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import bpy
import zsel

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLEND = os.path.join(ROOT, 'data/raw/zblend/Z-Anatomy/Startup.blend')
OUT = os.path.join(ROOT, 'data/raw/curves.json')


def gl(p):
    p = np.asarray(p, float)
    return np.stack([p[:, 0], p[:, 2], -p[:, 1]], 1)       # Blender Z-up -> glTF Y-up


def polylines(nv, edges):
    """Split an edge list into ordered vertex chains (one per spline)."""
    adj = [[] for _ in range(nv)]
    for a, b in edges:
        adj[a].append(b); adj[b].append(a)
    seen, out = np.zeros(nv, bool), []
    starts = [i for i in range(nv) if len(adj[i]) == 1] + list(range(nv))
    for s in starts:
        if seen[s] or not adj[s]:
            continue
        ch, cur, prev = [s], s, -1
        seen[s] = True
        while True:
            nx = [n for n in adj[cur] if n != prev and not seen[n]]
            if not nx:
                break
            prev, cur = cur, nx[0]
            seen[cur] = True
            ch.append(cur)
        if len(ch) > 1:
            out.append(ch)
    return out


def main():
    bpy.ops.wm.open_mainfile(filepath=BLEND)
    objs = [o for o in bpy.data.objects if zsel.selected(o)]
    for o in objs:
        if o.type == 'CURVE':
            zsel.lighten_curve(o)
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    used, res = set(), {}
    for o in objs:                                    # same naming/dedupe order as build_assets.extract
        me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), depsgraph=dg)
        if len(me.polygons) == 0:
            continue
        name, _ = zsel.clean_name(o.name)
        while name in used:
            name += ' 2'
        used.add(name)
        if o.type != 'CURVE':
            continue
        me.transform(o.matrix_world)
        vb = np.array([v.co[:] for v in me.vertices])
        c = o.data.copy()                              # same curve without bevel -> centreline edges
        c.bevel_depth = 0.0
        c.bevel_mode = 'ROUND'
        c.bevel_object = None
        c.extrude = 0.0
        c.resolution_u = max(c.resolution_u, 3)
        tmp = bpy.data.objects.new('tmp_' + name, c)
        bpy.context.scene.collection.objects.link(tmp)
        tmp.matrix_world = o.matrix_world
        dg.update()
        m2 = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg), depsgraph=dg)
        m2.transform(o.matrix_world)
        v = np.array([x.co[:] for x in m2.vertices])
        e = [tuple(x.vertices) for x in m2.edges]
        lines = [np.round(gl(v[ch]), 6).tolist() for ch in polylines(len(v), e)]
        res[name] = dict(lines=lines, mean=np.round(gl(vb).mean(0), 6).tolist())
        print(name, len(lines), 'lines', sum(len(l) for l in lines), 'pts')
    json.dump(res, open(OUT, 'w'))
    print('curves:', len(res), '->', OUT)


if __name__ == '__main__':
    main()
