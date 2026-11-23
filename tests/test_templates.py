# coding=utf-8
"""Static guards on the Knockout templates.

2026-11-17: the Probe here panel had `<!-- ko if -->` comments directly inside a <table>
around its <tr> rows. The browser's parser wraps the rows in an implied <tbody>, the closing
comment lands inside it and the opening one does not, and Knockout throws "Cannot find closing
comment tag" the moment a result appears. Rows inside an explicit <tbody> keep both comments
together.
"""
import glob
import os
import re

TEMPLATES = os.path.join(os.path.dirname(__file__), os.pardir, "octoprint_laserheightprobe", "templates")


def test_no_knockout_comment_sits_directly_in_a_table():
    bad = []
    for path in glob.glob(os.path.join(TEMPLATES, "*.jinja2")):
        with open(path) as handle:
            source = handle.read()
        for table in re.findall(r"<table\b.*?</table>", source, re.S):
            outside = re.sub(r"<(thead|tbody|tfoot)\b.*?</\1>", "", table, flags=re.S)
            if "<!-- ko" in outside or "<!-- /ko" in outside:
                bad.append(os.path.basename(path))
    assert bad == []


def test_knockout_comments_balance():
    for path in glob.glob(os.path.join(TEMPLATES, "*.jinja2")):
        with open(path) as handle:
            source = handle.read()
        assert len(re.findall(r"<!--\s*ko\b", source)) == len(re.findall(r"<!--\s*/ko\s*-->", source)), path


def test_every_name_a_binding_uses_exists():
    """A mistyped name in a data-bind stops Knockout binding the rest of the tab (nothing in the
    browser after it works). Every identifier must be a view model member, or a field of the item
    a foreach/with block binds (listed here), or JavaScript."""
    import os
    import re
    root = os.path.join(os.path.dirname(__file__), "..", "octoprint_laserheightprobe")
    js = open(os.path.join(root, "static", "js", "laserheightprobe.js")).read()
    members = set(re.findall(r"self\.([A-Za-z_][A-Za-z0-9_]*)\s*=", js))
    item_fields = {
        # foreach: rows / hereRows (results.table rows)
        "index", "x", "y", "z", "failed", "is_datum", "is_reference", "spread", "n", "taken", "height",
        "focus", "lightburn",
        # with: heightMap
        "points", "module", "source", "base", "range", "slope_deg", "worst", "rms", "tolerance", "warnings",
        # foreach: meshRows cells; uiChips; uiTabs; plan tiles; obstacles
        "text", "colour", "title", "label", "icon", "level", "value", "id", "view_lb", "box", "tone", "fix", "key",
        # foreach: burnRows (Recent burns)
        "src", "when", "detail", "status",
    }
    javascript = {"function", "return", "true", "false", "null", "undefined", "typeof", "new", "this", "var",
                  "Math", "String", "Number", "parseFloat", "parseInt", "window", "document", "event"}
    bad = []
    for name in ("laserheightprobe_tab.jinja2", "laserheightprobe_control.jinja2"):
        source = open(os.path.join(root, "templates", name)).read()
        bindings = re.findall(r'data-bind="([^"]*)"', source) + re.findall(r"<!--\s*ko\s+(.*?)-->", source)
        for binding in bindings:
            code = re.sub(r"'(?:\\.|[^'])*'", "''", binding)          # drop string literals
            for match in re.finditer(r"(?<![\w.$])([A-Za-z_][A-Za-z0-9_]*)(\s*:)?", code):
                word, colon = match.group(1), match.group(2)
                if colon and not code[match.end():].startswith(":"):
                    continue                                         # a binding name or object key
                if word in members or word in item_fields or word in javascript:
                    continue
                bad.append("%s: %s in %r" % (name, word, binding[:80]))
    assert bad == []
