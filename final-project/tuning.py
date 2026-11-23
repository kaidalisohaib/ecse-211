"""
Reads and edits the robot's tunable constants straight in the source files, for dashboard.py's Constants panel.

The .py files stay the only place the values live (SSH scripts, git and the simulator all see the same thing).
A constant is any top-level `NAME = <literal>` in snowplow_core.py, phase1_mapping.py or phase2_snow_removal.py;
its explanation is the comment next to it. Saving replaces only the value's text, so comments and layout stay as
they are. Every save keeps a copy of the old file in logs/constants_backup/ and a line in
logs/constants_changes.log.
"""
import ast
import io
import json
import os
import re
import shutil
import time
import tokenize

HERE = os.path.dirname(os.path.abspath(__file__))
FILES = [("core", "snowplow_core.py", "Robot core (snowplow_core.py)"),
         ("p1", "phase1_mapping.py", "Phase 1 (phase1_mapping.py)"),
         ("p2", "phase2_snow_removal.py", "Phase 2 (phase2_snow_removal.py)")]
CALIBRATION_KEYS = ("WHEEL_RADIUS", "LEFT_WHEEL_SCALE", "RIGHT_WHEEL_SCALE", "TRACK_WIDTH", "TRACK_WIDTH_CW",
                    "TRACK_WIDTH_CCW")
LOG_DIR = os.path.join(HERE, "logs")
_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")
_SECTION = re.compile(r"^#\s*(?:={3,}|-{3,})\s*(.*?)\s*(?:={3,}|-{3,})\s*$")
HIDDEN = {"HEADING_TO_DELTA", "ORIGIN_TILES", "TIGHT", "REL"}   # lookup tables / runtime state, not settings


def _path(file_id):
    for fid, name, _ in FILES:
        if fid == file_id:
            return os.path.join(HERE, name)
    raise KeyError(f"unknown file {file_id!r}")


def _comments(src):
    """{line number: comment text} for every comment in the file."""
    out = {}
    try:
        for tok in tokenize.generate_tokens(io.StringIO(src).readline):
            if tok.type == tokenize.COMMENT:
                out[tok.start[0]] = tok.string.lstrip("#").strip()
    except (tokenize.TokenError, IndentationError):
        pass
    return out


def _literal_assigns(tree):
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
                and _NAME.match(node.targets[0].id)):
            try:
                value = ast.literal_eval(node.value)
            except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError):
                continue
            yield node.targets[0].id, node, value


def _segment(lines_b, node):
    """Byte offsets (start, end) of node in the file (ast columns are UTF-8 byte offsets)."""
    starts = [0]
    for ln in lines_b:
        starts.append(starts[-1] + len(ln))
    return starts[node.lineno - 1] + node.col_offset, starts[node.end_lineno - 1] + node.end_col_offset


def list_constants(file_id):
    """[{"section", "items": [{"name", "value", "help", "line"}]}] in file order."""
    path = _path(file_id)
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    lines = src.splitlines()
    lines_b = src.encode("utf-8").splitlines(keepends=True)
    com = _comments(src)
    sections, current = [], {"section": "Top of the file", "items": []}
    section_at = {}
    for i, line in enumerate(lines, 1):
        m = _SECTION.match(line.strip())
        if m:
            section_at[i] = m.group(1)[:1].upper() + m.group(1)[1:].lower().replace("&", "and")
    names = {}
    for name, node, _ in _literal_assigns(tree):
        names[name] = names.get(name, 0) + 1
    last_line = 0
    for name, node, _ in _literal_assigns(tree):
        if names[name] > 1 or name in HIDDEN:
            continue                                   # assigned twice (ambiguous) or not a setting
        for ln in range(last_line + 1, node.lineno):
            if ln in section_at:
                if current["items"]:
                    sections.append(current)
                current = {"section": section_at[ln], "items": []}
        last_line = node.end_lineno
        a, b = _segment(lines_b, node.value)
        text = src.encode("utf-8")[a:b].decode("utf-8")
        help_parts = [com[ln] for ln in range(node.lineno, node.end_lineno + 1) if ln in com]
        ln = node.end_lineno + 1                       # comment-only continuation lines under it
        while ln <= len(lines) and lines[ln - 1].strip().startswith("#") and ln in com \
                and not _SECTION.match(lines[ln - 1].strip()) and lines[ln - 1].startswith(" "):
            help_parts.append(com[ln])
            ln += 1
        above, ln = [], node.lineno - 1                # comment block right above it (column 0: an indented
        while (ln >= 1 and lines[ln - 1].startswith("#")    # comment line continues the constant above)
               and not _SECTION.match(lines[ln - 1].strip())):
            above.insert(0, com.get(ln, ""))
            ln -= 1
        current["items"].append({"name": name, "value": text, "help": " ".join(above + help_parts).strip(),
                                 "line": node.lineno})
    if current["items"]:
        sections.append(current)
    return sections


def _compatible(old, new):
    if old is None or new is None:
        return True
    if isinstance(old, bool) or isinstance(new, bool):
        return isinstance(old, bool) and isinstance(new, bool)
    if isinstance(old, (int, float)) and isinstance(new, (int, float)):
        return True
    return type(old) is type(new)


def save_constants(file_id, changes):
    """changes = {NAME: new value text}. Returns [(name, old text, new text)]; raises ValueError on bad input
    (nothing is written then)."""
    path = _path(file_id)
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    lines_b = src.encode("utf-8").splitlines(keepends=True)
    found = {}
    for name, node, value in _literal_assigns(tree):
        found.setdefault(name, []).append((node, value))
    edits, done = [], []
    for name, text in changes.items():
        text = str(text).strip()
        if name not in found or len(found[name]) != 1:
            raise ValueError(f"{name}: not an editable constant in {os.path.basename(path)}")
        node, old = found[name][0]
        try:
            new = ast.literal_eval(text)
        except (ValueError, SyntaxError, TypeError, MemoryError, RecursionError):
            raise ValueError(f"{name}: {text!r} is not a value Python understands (numbers, True/False, "
                             f"'text', (tuples), [lists], {{dicts}})")
        if not _compatible(old, new):
            raise ValueError(f"{name}: was {type(old).__name__}, got {type(new).__name__} ({text})")
        a, b = _segment(lines_b, node.value)
        old_text = src.encode("utf-8")[a:b].decode("utf-8")
        if old_text != text:
            edits.append((a, b, text))
            done.append((name, old_text, text))
    if not edits:
        return []
    data = src.encode("utf-8")
    for a, b, text in sorted(edits, reverse=True):
        data = data[:a] + text.encode("utf-8") + data[b:]
    new_src = data.decode("utf-8")
    check = {n: v for n, _, v in _literal_assigns(ast.parse(new_src))}   # must still be valid Python
    for name, _, text in done:
        if check.get(name) != ast.literal_eval(text):
            raise ValueError(f"{name}: the edit didn't come out right; nothing saved")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    os.makedirs(os.path.join(LOG_DIR, "constants_backup"), exist_ok=True)
    shutil.copy2(path, os.path.join(LOG_DIR, "constants_backup", f"{os.path.basename(path)}.{stamp}"))
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(new_src)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
    with open(os.path.join(LOG_DIR, "constants_changes.log"), "a", encoding="utf-8") as f:
        for name, old_text, text in done:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {os.path.basename(path)}  {name}: {old_text} -> {text}\n")
    return done


def read_calibration(path):
    """(exists, {key: value}) of a calibration_params.json."""
    try:
        with open(path) as f:
            data = json.load(f)
        return True, {k: data[k] for k in CALIBRATION_KEYS if k in data}
    except (OSError, ValueError):
        return False, {}


def save_calibration(path, values):
    """Writes the given keys (numbers) into calibration_params.json, keeping any other keys in it."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        data = {}
    clean = {}
    for k, v in values.items():
        if k not in CALIBRATION_KEYS:
            raise ValueError(f"{k}: not a calibration value")
        try:
            clean[k] = float(v)
        except (TypeError, ValueError):
            raise ValueError(f"{k}: {v!r} is not a number")
        if not 0 < clean[k] < 100:
            raise ValueError(f"{k}: {v} is out of range")
    data.update(clean)
    data["TIMESTAMP"] = time.strftime("%Y-%m-%d %H:%M:%S") + " (edited on the dashboard)"
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(os.path.join(LOG_DIR, "constants_changes.log"), "a", encoding="utf-8") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {path}  " +
                ", ".join(f"{k}={v}" for k, v in clean.items()) + "\n")
