#!/usr/bin/env python3
"""Check platform/port_settings_index.inc against the F1 overlay sources.

Usage: check_settings_index.py <repo root>

Checks, each problem printed on its own line:
  (a) every row parses, its page is a kDebugPages name and its sub-tab is a
      SubTab("page", "sub") pair in debug_ui.cpp ("" when the page has none);
  (b) every row's label appears as a string literal in platform/**/*.cpp
      ("label" or "label##...");
  (c) every ImGui widget in debug_ui.cpp whose label is a literal with visible
      text (before "##") is a label in the index or listed in EXCLUDED.

Duplicate (page, sub-tab, label) groups are reported as notes; they are legal
when the same widget is drawn on two paths, so they do not fail the check.
Exit status: 0 when the index passes, 1 with problems, 2 on bad usage.
"""

import pathlib
import re
import sys

# Widgets in debug_ui.cpp that check (c) skips: they are not F1 settings.
EXCLUDED = {
    # Photo mode's floating panel (DrawPhotoUI), not an F1 page.
    "FOV": "photo mode panel, not an F1 page",
    "Roll": "photo mode panel, not an F1 page",
    "%dx": "photo mode shot-size radio, not an F1 page",
}

INDEX = pathlib.Path("platform/port_settings_index.inc")
DEBUG_UI = pathlib.Path("platform/debug_ui.cpp")
# debug_ui.cpp #includes its parts from here, in place.
DEBUG_UI_PARTS = pathlib.Path("platform/debug_ui")


def debug_ui_sources(root):
    """debug_ui.cpp and its parts, as (path, text) in file order."""
    files = [(DEBUG_UI, (root / DEBUG_UI).read_text(encoding="utf-8"))]
    for part in sorted((root / DEBUG_UI_PARTS).glob("*.inc")):
        files.append((part.relative_to(root), part.read_text(encoding="utf-8")))
    return files

STR = r'"((?:[^"\\]|\\.)*)"'
ROW_RE = re.compile(r"^\{\s*" + STR + r"\s*,\s*" + STR + r"\s*,\s*" + STR + r"\s*,\s*" + STR
                    + r"\s*,\s*" + STR + r"\s*\},?\s*$")
PAGE_RE = re.compile(r'\{\s*"([^"]+)"\s*,\s*Draw\w+\s*\}')
SUBTAB_RE = re.compile(r'SubTab\(\s*"([^"]+)"\s*,\s*"([^"]+)"\s*\)')
WIDGET_RE = re.compile(
    r'ImGui::(Checkbox|SliderInt|SliderFloat\w*|DragInt\w*|DragFloat\w*|Combo|BeginCombo'
    r'|RadioButton|InputText\w*|ColorEdit\w*)\(\s*' + STR)


def visible(label):
    return label.split("##", 1)[0]


def main(argv):
    if len(argv) != 2:
        print("usage: check_settings_index.py <repo root>", file=sys.stderr)
        return 2
    root = pathlib.Path(argv[1])
    problems = []

    debug_files = debug_ui_sources(root)
    debug_src = "\n".join(text for _path, text in debug_files)
    pages = set(PAGE_RE.findall(debug_src))
    subtabs = {}
    for page, sub in SUBTAB_RE.findall(debug_src):
        subtabs.setdefault(page, set()).add(sub)

    rows = []
    for lineno, line in enumerate((root / INDEX).read_text(encoding="utf-8").splitlines(), 1):
        if not line.startswith("{"):
            continue
        m = ROW_RE.match(line)
        if not m:
            problems.append(f"{INDEX}:{lineno}: row does not parse: {line.strip()}")
            continue
        rows.append((lineno,) + m.groups())

    if not pages:
        problems.append(f"{DEBUG_UI}: no kDebugPages entries found")

    for lineno, page, sub, section, label, keywords in rows:
        if page not in pages:
            problems.append(f"{INDEX}:{lineno}: unknown page '{page}' (not in kDebugPages)")
        if sub and sub not in subtabs.get(page, set()):
            problems.append(f"{INDEX}:{lineno}: sub-tab '{sub}' is not a SubTab() of page '{page}'")
        if not label:
            problems.append(f"{INDEX}:{lineno}: empty label")

    sources = "\n".join(
        [p.read_text(encoding="utf-8", errors="replace")
         for p in sorted((root / "platform").rglob("*.cpp"))] + [text for _path, text in debug_files[1:]])
    for lineno, _page, _sub, _section, label, _keywords in rows:
        if f'"{label}"' not in sources and f'"{label}##' not in sources:
            problems.append(f"{INDEX}:{lineno}: label '{label}' is not a string literal in platform/**/*.cpp")

    index_labels = {row[4] for row in rows}
    for path, text in debug_files:
        for m in WIDGET_RE.finditer(text):
            label = visible(m.group(2))
            if not label or label in index_labels or label in EXCLUDED:
                continue
            line = text.count("\n", 0, m.start()) + 1
            problems.append(f"{path}:{line}: ImGui::{m.group(1)} '{label}' is not in {INDEX} "
                            "and not in EXCLUDED")

    groups = {}
    for lineno, page, sub, _section, label, _keywords in rows:
        groups.setdefault((page, sub, label), []).append(lineno)
    for (page, sub, label), linenos in groups.items():
        if len(linenos) > 1:
            print(f"note: duplicate (page, sub-tab, label) {(page, sub, label)} at lines "
                  f"{', '.join(map(str, linenos))}")

    for problem in problems:
        print(problem)
    if problems:
        print(f"FAIL: {len(problems)} problem(s) in {len(rows)} rows")
        return 1
    print(f"OK: {len(rows)} rows, {len(EXCLUDED)} excluded labels")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
