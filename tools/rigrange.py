"""Joint ranges (assets/joints.json, user-editable) and dof limits; thumb CMC limits from collisions."""
import json, os
import numpy as np
import rigpose as P

PATH = os.path.join(P.ROOT, 'assets/joints.json')
DEFAULT = {
    "_note": "Typical adult normal ranges in degrees (edit freely). Positive = flexion, pronation, "
             "ulnar deviation, radial deviation of MCP (abd), CMC1 flexion/palmar abduction.",
    "forearm": {"pronation": 80, "supination": 80},
    "wrist": {"flexion": 80, "extension": 70, "radial": 20, "ulnar": 30},
    "mcp": {"flexion": 90, "hyperextension": 20, "abd_at_0": 20, "abd_at_90": 5},
    "pip": {"flexion": 110, "hyperextension": 0},
    "dip": {"flexion": 80, "hyperextension": 10},
    "thumb_mcp": {"flexion": 50, "hyperextension": 0},
    "thumb_ip": {"flexion": 80, "hyperextension": 15},
    "cmc_cup": {"ring": 10, "little": 20},
    "active_dip_per_pip": 0.667,
}
CMC1_ARC = {"cmc1_fe": 53, "cmc1_aa": 42}   # total arcs, Cooney et al. JBJS Am 1981 (cap on the free side)
CMC1_NEAR = ['Trapezium bone', 'Trapezoid bone', 'Second metacarpal bone', 'Scaphoid bone']


def limits(J, dof, pose=None):
    """(min, max) anatomical degrees for one dof; MCP abd range shrinks with flexion."""
    if dof == 'pronation':
        return -J['forearm']['supination'], J['forearm']['pronation']
    if dof == 'wrist_fe':
        return -J['wrist']['extension'], J['wrist']['flexion']
    if dof == 'wrist_ru':
        return -J['wrist']['radial'], J['wrist']['ulnar']
    if dof.endswith('_abd'):
        f = (pose or {}).get(dof.replace('_abd', '_flex'), 0)
        t = min(max(f, 0), 90) / 90
        m = J['mcp']['abd_at_0'] + t * (J['mcp']['abd_at_90'] - J['mcp']['abd_at_0'])
        return -m, m
    if dof == 'cmc4':
        return 0, J['cmc_cup']['ring']
    if dof == 'cmc5':
        return 0, J['cmc_cup']['little']
    if dof.startswith('cmc1'):
        k = J['thumb_cmc'][dof]
        return -k['neg'], k['pos']
    key = {'mcp1': 'thumb_mcp', 'ip1': 'thumb_ip'}.get(dof) or \
        ('mcp' if dof.startswith('mcp') else 'pip' if dof.startswith('pip') else 'dip')
    return -J[key]['hyperextension'], J[key]['flexion']


def overlap_mesh(B, names):
    return P.Overlap({n: B.all(n) for n in names})


def sweep(rig, cm, ov, dof, sgn, base_depth, cap=70, step=2.0):
    """First angle (deg) along sgn where MC1 overlaps a neighbour by > 0.5 mm beyond rest."""
    a = 0.0
    while a < cap:
        a += step
        W = P.solve(rig, cm, {dof: sgn * a})
        for n in CMC1_NEAR:
            if ov.depth('First metacarpal bone', W['First metacarpal bone'], n, W[n]) > base_depth[n] + 0.5e-3:
                return a - step, n
    return None, None


def write_joints(B, rig, log):
    J = json.load(open(PATH)) if os.path.exists(PATH) else json.loads(json.dumps(DEFAULT))
    if 'thumb_cmc' not in J or J['thumb_cmc'].get('_auto'):
        cm = json.load(open(os.path.join(P.ROOT, 'assets/carpal_model.json')))
        ov = overlap_mesh(B, CMC1_NEAR + ['First metacarpal bone'])
        W = P.solve(rig, cm, {})
        base = {n: ov.depth('First metacarpal bone', W['First metacarpal bone'], n, W[n]) for n in CMC1_NEAR}
        tc = {"_auto": True, "_note": "from rest pose; collision side = bones touch (+0.5 mm); "
              "free side = total arc (Cooney 1981: FE 53, AA 42) minus collision side, min 10"}
        for dof in ('cmc1_fe', 'cmc1_aa'):
            r = {}
            for key, sgn in (('pos', 1), ('neg', -1)):
                r[key] = sweep(rig, cm, ov, dof, sgn, base)
            arc = CMC1_ARC[dof]
            hit = {k: v[0] for k, v in r.items() if v[0] is not None}
            out = {}
            for k in ('pos', 'neg'):
                o = 'neg' if k == 'pos' else 'pos'
                if k in hit:
                    out[k] = round(hit[k])
                elif o in hit:
                    out[k] = round(max(arc - hit[o], 10))
                else:
                    out[k] = round(arc / 2)
            tc[dof] = out
            log('%s limits: +%d / -%d deg (collision: %s)' % (
                dof, out['pos'], out['neg'], {k: v[1] for k, v in r.items() if v[1]}))
        J['thumb_cmc'] = tc
    json.dump(J, open(PATH, 'w'), indent=1)
    return J
