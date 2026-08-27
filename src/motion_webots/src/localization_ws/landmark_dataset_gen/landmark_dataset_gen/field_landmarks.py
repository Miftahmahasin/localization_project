#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shim — the field landmark model moved to the shared ``landmark_geometry``.

The authoritative implementation now lives in
``landmark_geometry/landmark_geometry/field_landmarks.py`` so the label
generator (this package) and the runtime localization backend import the SAME
map/geometry. Do NOT add geometry here — edit the shared package instead.
"""
from landmark_geometry.field_landmarks import *  # noqa: F401,F403
