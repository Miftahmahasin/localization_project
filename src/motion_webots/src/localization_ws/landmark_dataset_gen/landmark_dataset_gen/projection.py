#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shim — the camera projection moved to the shared ``landmark_geometry``.

The authoritative implementation now lives in
``landmark_geometry/landmark_geometry/projection.py`` (forward world->pixel +
inverse ``unproject_to_ground``). The label generator and the runtime
localization backend import the SAME projector so the geometry used to label the
data is byte-for-byte the geometry used at inference. Do NOT reimplement the
camera model here — edit the shared package instead.
"""
from landmark_geometry.projection import *  # noqa: F401,F403
