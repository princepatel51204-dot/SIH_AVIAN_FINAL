"""Structural check on the FeatureScript sources.

This is not an Onshape compiler -- there isn't one outside Onshape. It checks
the things that actually break a paste-in: unbalanced delimiters, a missing
version header, features that never close, precondition/body mismatch, and
references to variables that the Variable Studio does not define.
"""
from __future__ import annotations
import os
import re
import sys

DIR = "pkg/01_Onshape_FeatureScript"


def declared_vars(path):
    src = open(path).read()
    return set(re.findall(r"export const (\w+)\s*=", src))


def check(path, known):
    src = open(path).read()
    errs = []
    name = os.path.basename(path)

    if not src.startswith("FeatureScript "):
        errs.append("missing FeatureScript version header")
    if 'import(path : "onshape/std/geometry.fs"' not in src:
        errs.append("missing geometry.fs import")

    # strip strings and comments before counting delimiters
    s = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    s = re.sub(r'//[^\n]*', '', s)
    s = re.sub(r'"(\\.|[^"\\])*"', '""', s)
    for op, cl, lab in (("{", "}", "braces"), ("(", ")", "parens"),
                        ("[", "]", "brackets")):
        d = s.count(op) - s.count(cl)
        if d:
            errs.append(f"unbalanced {lab}: {d:+d}")

    if name.endswith(".fs"):
        nfeat = len(re.findall(r"defineFeature\(function\(", s))
        # search the ORIGINAL source: the annotation name lives in a
        # string literal, which the delimiter-safe copy has blanked out
        nann = len(re.findall(r'annotation \{ "Feature Type Name"', src))
        if nfeat == 0:
            errs.append("no defineFeature")
        if nann != nfeat:
            errs.append(f"{nann} feature annotations for {nfeat} features")
        if s.count("precondition") != nfeat:
            errs.append("precondition count != feature count")
        for m in re.finditer(r"defineFeature", s):
            pass
        tail = s.rstrip()
        if not (tail.endswith(";") or tail.endswith("}")):
            errs.append("file ends mid-statement")

    # every getVariable name must exist in the Variable Studio
    used = set(re.findall(r'getVariable\(context,\s*"(\w+)"\)', src))
    missing = sorted(used - known)
    if missing:
        errs.append("undefined variables: " + ", ".join(missing))

    # every enum referenced must be declared in the same file
    enums = set(re.findall(r"export enum (\w+)", s))
    for e in set(re.findall(r"definition\.\w+ is (\w+);", s)):
        if e[0].isupper() and e not in enums and e not in (
                "boolean", "Context", "Id"):
            errs.append(f"enum {e} used but not declared in this file")
    return errs, len(used)


def main():
    varfile = os.path.join(DIR, "AVIAN_Variables.txt")
    known = declared_vars(varfile)
    print(f"Variable Studio declares {len(known)} variables\n")
    bad = 0
    for fn in sorted(os.listdir(DIR)):
        p = os.path.join(DIR, fn)
        if not fn.endswith((".fs", ".txt")):
            continue
        errs, nused = check(p, known)
        st = "OK  " if not errs else "FAIL"
        print(f"  {st} {fn:34s} {nused:3d} variables referenced")
        for e in errs:
            print(f"         !! {e}")
            bad += 1
    print()
    print("PASS" if bad == 0 else f"{bad} problems")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
