"""Phase E3: animated presets for js/move.js -> assets/presets.json (edit freely; re-run to rebuild).
Thumb contact poses (tip/key pinch, Kapandji 1-10, thumb in fist) are optimised on the real bone meshes like the E1
opposition check (tools/check_rig.py): thumb pulp to a target point at a soft-tissue gap, pulps facing, no bone overlap.
Key format: partial pose in degrees; fingers/thumb not listed = 0, forearm/wrist not listed = unchanged;
"link" = DIP follows PIP (active ratio). 'from' = loop partner (default: open hand). 'cycle' = loop through keys only.
Run: python3 tools/make_presets.py   (needs data/raw/hand_raw.glb)"""
import json, os, sys
import numpy as np
from scipy.optimize import minimize
sys.path.insert(0, os.path.dirname(__file__))
import check_rig as CR
from build_rig import ph, mc

P, rig, cm, B, OV = CR.P, CR.rig, CR.cm, CR.B, CR.OV
RAD, VOL = -CR.ULN, CR.VOL
THUMB_D = ['cmc1_fe', 'cmc1_aa', 'mcp1', 'ip1']
LOG = []


def surf(bone, frac, direction):
    """Bone surface point (rest coords) at fraction frac (0 = proximal, 1 = distal) along the bone, most in direction."""
    v = B.verts(bone)
    ax = np.linalg.svd(v - v.mean(0))[2][0]
    ax = ax if (v @ ax)[np.argmin(v @ CR.DIS)] < (v @ ax)[np.argmax(v @ CR.DIS)] else -ax
    s = (v - v.mean(0)) @ ax
    t = (s - s.min()) / (s.max() - s.min())
    near = v[np.abs(t - frac) < 0.12]
    return near[np.argmax(near @ direction)], np.asarray(direction, float)


def touch(base, bone, p, n, gap, others):
    """Thumb dofs so the thumb pulp touches point p (normal n) of bone, at gap (m)."""
    tp, tn, tb = CR.PULP[1]

    def pose_of(x):
        return CR.clamp(dict(base, **dict(zip(THUMB_D, x))))

    def cost(x):
        W = P.solve(rig, cm, pose_of(x))
        pt, nt = P.apply(W[tb], tp[None])[0], W[tb][:3, :3] @ tn
        pf, nf = P.apply(W[bone], p[None])[0], W[bone][:3, :3] @ n
        hit = max(OV.depth(a, W[a], b, W[b]) for a in CR.THUMB for b in others)
        return ((np.linalg.norm(pt - pf) - gap) * 1e3) ** 2 + 30 * (1 + nt @ nf) ** 2 + (hit * 2e3) ** 2
    best = None
    for x0 in ((10, 10, 20, 20), (20, -20, 30, 30), (-10, 30, 10, 10), (25, -24, 45, 60)):
        r = minimize(cost, x0, method='Powell', options=dict(maxiter=3000, xtol=0.1, ftol=1e-4))
        if best is None or r.fun < best.fun:
            best = r
    pose = pose_of(best.x)
    W = P.solve(rig, cm, pose)
    d = np.linalg.norm(P.apply(W[tb], tp[None])[0] - P.apply(W[bone], p[None])[0]) * 1e3
    return {k: round(pose[k], 1) for k in THUMB_D}, d


def fingers(mcp, pip, dip='link', which=(2, 3, 4, 5)):
    out = {}
    for f in which:
        out.update({'mcp%d_flex' % f: mcp, 'pip%d' % f: pip, 'dip%d' % f: dip})
    return out


def main():
    Z = {d: 0.0 for d in rig['dofs']}
    relax = fingers(30, 40, 'link', (3, 4, 5))
    pre = [dict(name='Neutral', keys=[dict(Z)]),
           dict(name='Full fist', keys=[dict(fingers(90, 110), cmc4=10, cmc5=20, cmc1_fe=0, cmc1_aa=20, mcp1=50, ip1=30)]),  # E4: thumb clear of the fingers
           dict(name='Hook fist', keys=[fingers(0, 110)]),
           dict(name='Straight fist', keys=[fingers(90, 110, 0)]),
           dict(name='Intrinsic plus', keys=[dict(fingers(90, 0, 0), cmc1_aa=30)]),
           dict(name='Claw (intr. minus)', keys=[fingers(-20, 90)])]
    # tip pinch = pulp-to-pulp opposition with the index (E1 optimiser), other fingers relaxed
    tip, d, a = CR.opposition(2)
    pre.append(dict(name='Tip pinch', keys=[dict(relax, **{k: round(tip[k], 1) for k in THUMB_D + ['mcp2_flex', 'mcp2_abd', 'pip2', 'dip2']})]))
    LOG.append('tip pinch: pulp gap %.1f mm, pulps %.0f deg' % (d, a))
    # key pinch: thumb pulp on the radial side of the index middle phalanx, fingers flexed
    base = CR.clamp(dict(Z, **fingers(80, 95, 60, (3, 4, 5)), mcp2_flex=55, pip2=70, dip2=35))
    p, n = surf(ph('Middle', 2), 0.5, RAD)
    th, d = touch(base, ph('Middle', 2), p, n, 7e-3, [ph('Middle', 2), ph('Proximal', 2), mc(2)])
    pre.append(dict(name='Key pinch', keys=[dict({k: v for k, v in base.items() if v and k not in ('pronation', 'wrist_fe', 'wrist_ru')}, **th)]))
    LOG.append('key pinch: gap to target %.1f mm (aim 7)' % d)
    # thumb in fist (Finkelstein): thumb pulp to the palm over the 4th metacarpal head, then fist + ulnar deviation
    p, n = surf(mc(4), 0.85, VOL)
    th, d = touch(dict(Z), mc(4), p, n, 12e-3, [mc(2), mc(3), mc(4)])
    fk = dict(fingers(90, 100), cmc4=10, cmc5=20, **th)
    pre.append(dict(name='Finkelstein', keys=[fk, dict(fk, wrist_ru=30)], cycle=True))
    LOG.append('thumb in palm (Finkelstein): gap %.1f mm (aim 12)' % d)
    pre += [dict(name='Phalen', keys=[dict(fingers(25, 35), wrist_fe=80)], **{'from': dict(wrist_fe=0)}),
            dict(name='Reverse Phalen', keys=[dict(wrist_fe=-70)], **{'from': dict(wrist_fe=0)}),
            dict(name="Dart-thrower's", keys=[dict(fingers(20, 25), wrist_fe=-35, wrist_ru=-15),
                                               dict(fingers(20, 25), wrist_fe=45, wrist_ru=25)], cycle=True),
            dict(name='Pronation-supination', keys=[dict(pronation=80), dict(pronation=-80)], cycle=True)]
    # Kapandji opposition score 1-10 (Kapandji 1986): thumb tip to ...
    targets = {1: (ph('Proximal', 2), 0.5, RAD, 7e-3), 2: (ph('Middle', 2), 0.5, RAD, 7e-3),
               7: (ph('Middle', 5), 0.95, VOL, 7e-3), 8: (ph('Proximal', 5), 0.95, VOL, 7e-3),
               9: (ph('Proximal', 5), 0.2, VOL, 8e-3), 10: (mc(5), 0.85, VOL, 12e-3)}
    for k in range(1, 11):
        if 3 <= k <= 6:                                       # pulp to pulp of index..little (finger may flex)
            f = k - 1
            pose, d, a = CR.opposition(f)
            key = {kk: round(v, 1) for kk, v in pose.items() if kk in THUMB_D or kk.endswith(str(f)) and v}
            LOG.append('Kapandji %d: pulp gap %.1f mm, pulps %.0f deg' % (k, d, a))
        else:
            bone, fr, dirn, gap = targets[k]
            p, n = surf(bone, fr, dirn)
            f = 2 if k <= 2 else 5
            others = [ph('Proximal', f), ph('Middle', f), ph('Distal', f), mc(f)]
            key, d = touch(dict(Z), bone, p, n, gap, others)
            LOG.append('Kapandji %d: gap %.1f mm (aim %.0f)' % (k, d, gap * 1e3))
        pre.append(dict(name='Kapandji %d' % k, keys=[key]))
    out = {'_note': __doc__.split('\n')[0] + ' Key format: see tools/make_presets.py.', 'log': LOG, 'presets': pre}
    json.dump(out, open(os.path.join(CR.ROOT, 'assets/presets.json'), 'w'), indent=1)
    print('\n'.join(LOG))


if __name__ == '__main__':
    main()
