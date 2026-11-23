"""Select right upper-limb structures in Z-Anatomy and assign layers/names."""
import re
from mathutils import Vector

MID_HUMERUS_Z = 1.246          # metres, Z-Anatomy world (humerus spans 1.087-1.405)
RIGHT = re.compile(r'\.[a-z0-9]*r$')
UPPER_LIMB = re.compile(r'upper limb|hand|forearm|arm\b|digit|finger|thumb|wrist|carp|'
                        r'palm|brachi|radial|ulnar|median|cubit', re.I)
EXCLUDE = re.compile(r'pectoral|thorac|phrenic|scapul|lymph|node', re.I)
FASCIA = re.compile(r'fascia|aponeurosis|retinaculum|septum|fibrous shea|'
                    r'superficial transverse metacarpal', re.I)
SHEATH = re.compile(r'sheath|bursa', re.I)
VEIN = re.compile(r'vein|venous', re.I)


def system(o):
    for c in o.users_collection:
        m = re.match(r'^(\d):', c.name)
        if m:
            return int(m.group(1))
    return 0


def world_bbox(o):
    bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
    return ([min(v[i] for v in bb) for i in range(3)],
            [max(v[i] for v in bb) for i in range(3)])


def context_text(o):
    s = [c.name for c in o.users_collection]
    p = o.parent
    while p:
        s.append(p.name)
        p = p.parent
    return ' '.join(s) + ' ' + o.name


def selected(o):
    if o.type not in ('MESH', 'CURVE') or not RIGHT.search(o.name):
        return False
    if system(o) not in (1, 2, 3, 4, 5, 7, 9) or EXCLUDE.search(o.name):
        return False
    mn, mx = world_bbox(o)
    if mn[0] > -0.05 or mx[2] < 0.6 or mn[2] > MID_HUMERUS_Z or (mn[2] + mx[2]) / 2 > 1.30:
        return False
    return bool(UPPER_LIMB.search(context_text(o)))


def layer(o):
    s, n = system(o), o.name
    if s == 1:
        return 'bone'
    if s == 2:
        return 'attachment'
    if s == 3:
        return 'ligament_capsule'
    if s == 4:
        if FASCIA.search(n):
            return 'fascia_retinacula'
        if SHEATH.search(n):
            return 'tendon_sheath'
        return 'muscle'
    if s == 5:
        return 'vein' if VEIN.search(n) else 'artery'
    if s == 7:
        return 'nerve'
    return 'skin'


def clean_name(n):
    """'Median nerve.r' -> 'Median nerve'; 'FDS.e2r' -> ('FDS (insertion 2)', 'insertion')."""
    base, suf = n.rsplit('.', 1)
    base = base.strip()
    if base.startswith('(') and base.endswith(')'):
        base = base[1:-1]
    kind = None
    m = re.match(r'^([oe])(\d*)r$', suf)
    if m:
        kind = 'origin' if m.group(1) == 'o' else 'insertion'
        base = '%s (%s%s)' % (base, kind, ' ' + m.group(2) if m.group(2) else '')
    return base.replace(':', '-'), kind


def lighten_curve(o):
    """Lower curve/bevel resolution so vessels and nerves stay light."""
    c = o.data
    c.resolution_u = min(c.resolution_u, 3)
    c.bevel_resolution = min(c.bevel_resolution, 2)
    if getattr(c, 'bevel_mode', '') == 'OBJECT' and c.bevel_object:
        bo = c.bevel_object.data
        bo.resolution_u = min(bo.resolution_u, 2)
