#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Class-id definitions and remapping for the landmark detector.

The trained YOLOv8n uses the DATASET ordering (landmark_dataset_gen):

    0 = L              (line L-corner)
    1 = T              (line T-junction)
    2 = X              (line X-cross / penalty & centre marks)
    3 = goalpost
    4 = center_circle

Downstream soccerbot code (`soccer_object_detection.object_detect_node.Label`
and the crossing consumers) uses a DIFFERENT ordering:

    BALL=0 GOALPOST=1 ROBOT=2 L_INTERSECTION=3 T_INTERSECTION=4 X_INTERSECTION=5 TOPBAR=6

So the inference node must remap the published class id. The mapping is a
parameter; two presets are provided plus a free-form "a:b,c:d" string. A target
id of -1 means "drop this class" (do not publish it).
"""

# dataset (model output) ids -> canonical names
DATASET_NAMES = {0: 'L', 1: 'T', 2: 'X', 3: 'goalpost', 4: 'center_circle'}

# preset target-id maps (dataset id -> published id)
PRESET_IDENTITY = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4}
# match soccerbot Label enum; center_circle has no equivalent -> dropped (-1)
PRESET_SOCCERBOT = {0: 3, 1: 4, 2: 5, 3: 1, 4: -1}

_PRESETS = {'identity': PRESET_IDENTITY, 'soccerbot': PRESET_SOCCERBOT}


def parse_class_map(spec):
    """Parse a class-map parameter into {dataset_id: published_id}.

    `spec` may be a preset name ('identity' | 'soccerbot') or a comma list of
    'src:dst' pairs, e.g. '0:3,1:4,2:5,3:1,4:-1'. Unknown/empty -> identity.
    """
    if spec is None:
        return dict(PRESET_IDENTITY)
    spec = str(spec).strip().lower()
    if spec in _PRESETS:
        return dict(_PRESETS[spec])
    out = dict(PRESET_IDENTITY)
    if not spec:
        return out
    try:
        for pair in spec.split(','):
            pair = pair.strip()
            if not pair:
                continue
            src, dst = pair.split(':')
            out[int(src)] = int(dst)
    except ValueError:
        # malformed -> fall back to identity rather than crash the node
        return dict(PRESET_IDENTITY)
    return out
