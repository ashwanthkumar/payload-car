# -*- coding: utf-8 -*-
"""Fusion entry point. Utilities -> Add-Ins -> Scripts -> add this folder -> Run.

Builds the parametric RC payload car and exports payload_car.f3d next to this script.
The actual modelling lives in builder.py (the single source of truth shared with the MCP path).
"""

import os
import traceback

import adsk.core

import builder


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = builder.build(app)
        try:
            out = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'payload_car.f3d')
            builder.export_f3d(design, out)
            note = '\nExported: ' + out
        except Exception:
            note = '\n(Could not auto-export .f3d - use File > Export manually.)'
        ui.messageBox('RC payload car built.\nEdit Modify > Change Parameters to regenerate.' + note)
    except Exception:
        ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
