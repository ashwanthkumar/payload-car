# -*- coding: utf-8 -*-
"""Command-line interface, runnable with uv.

    uv run payload-car validate     # offline check of every parameter expression
    uv run payload-car params       # print the parameter table
    uv run payload-car bootstrap    # emit the Fusion/MCP script that builds the model

The geometry itself is created inside Fusion (the `adsk` API only exists there); this CLI
covers everything that can run under plain CPython/uv.
"""

import argparse
import ast
import os
import re
import sys

from . import builder

_BOOTSTRAP = '''import adsk.core

def run(_context):
    app = adsk.core.Application.get()
    path = {path!r}
    ns = {{'__file__': path}}   # plain exec does not define __file__; assemble.py needs it
    with open(path) as f:
        exec(compile(f.read(), path, 'exec'), ns)
    design = ns['assemble'](app)
    bodies = sum(c.bRepBodies.count for c in design.allComponents)
    print('built %d components, %d bodies' % (design.allComponents.count, bodies))
'''


def _geometry_expressions():
    """Pull every parameter-only math expression out of builder.py for offline checking.

    Uses ast to read string literals (robust to apostrophes in comments/docstrings).
    """
    src = open(builder.__file__, encoding='utf-8').read()
    keys = set(builder.P)
    found = set()
    for node in ast.walk(ast.parse(src)):
        if not (isinstance(node, ast.Constant) and isinstance(node.value, str)):
            continue
        s = node.value
        toks = re.findall(r'[A-Za-z_][A-Za-z0-9_]*', s)
        if (toks and all(t in keys for t in toks) and re.fullmatch(r'[A-Za-z0-9_+\-*/(). ]+', s)
                and s.count('(') == s.count(')')):  # skip concatenation fragments (unbalanced parens)
            found.add(s)
    return found


def cmd_validate(_args):
    exprs = _geometry_expressions()
    bad = []
    for e in sorted(exprs):
        try:
            eval(e, {'__builtins__': {}}, builder.P)  # noqa: S307 - trusted internal expressions
        except Exception as ex:  # noqa: BLE001
            bad.append((e, str(ex)))
    print('parameters:           {}'.format(len(builder.P)))
    print('geometry expressions: {} checked, {} failed'.format(len(exprs), len(bad)))
    for e, msg in bad:
        print('  FAIL  {}  ->  {}'.format(e, msg))
    print('OK' if not bad else 'FAILED')
    return 1 if bad else 0


def cmd_params(_args):
    for title, params in (('car', builder.PARAMS), ('rocker suspension', builder.ROCKER_PARAMS)):
        print('--- {} ---'.format(title))
        for k, (val, comment) in params.items():
            print('{:22} {:>8} mm   {}'.format(k, val, comment))
    return 0


def cmd_bootstrap(_args):
    here = os.path.dirname(os.path.abspath(builder.__file__))
    print(_BOOTSTRAP.format(path=os.path.join(here, 'assemble.py')))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog='payload-car',
                                 description='Parametric RC payload car for Autodesk Fusion')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('validate', help='offline check of every parameter expression')
    sub.add_parser('params', help='print the parameter table')
    sub.add_parser('bootstrap', help='emit the Fusion/MCP script that builds the model')
    args = ap.parse_args(argv)
    return {'validate': cmd_validate, 'params': cmd_params, 'bootstrap': cmd_bootstrap}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
