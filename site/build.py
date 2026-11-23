#!/usr/bin/env python3
"""Build the GitHub Pages site into _site/: the static page, the films, and the user guide and
README rendered from the repository's Markdown with pandoc (so the docs have one source)."""
import os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SITE = os.path.join(ROOT, "site")
OUT = os.path.join(ROOT, "_site")
REPO = "https://github.com/chrisns/octoburn"


def links(html, base):
    """Repository-relative links become site pages or GitHub URLs."""
    def fix(m):
        attr, url = m.group(1), m.group(2)
        if re.match(r"^(https?:|mailto:|#|data:)", url):
            return m.group(0)
        path = os.path.normpath(os.path.join(base, url.split("#")[0])) if url.split("#")[0] else ""
        frag = ("#" + url.split("#", 1)[1]) if "#" in url else ""
        if path == "docs/USER_GUIDE.md":
            return '%s="guide.html%s"' % (attr, frag)
        if path == "README.md":
            return '%s="reference.html%s"' % (attr, frag)
        if path.startswith("docs/img/"):
            return '%s="img/guide/%s"' % (attr, path[len("docs/img/"):])
        if path.startswith("octoprint_laserheightprobe/static/img/"):
            return '%s="img/%s"' % (attr, os.path.basename(path))
        kind = "tree" if (url.endswith("/") or os.path.isdir(os.path.join(ROOT, path))) else "blob"
        return '%s="%s/%s/main/%s%s"' % (attr, REPO, kind, path, frag)
    return re.sub(r'(href|src)="([^"]+)"', fix, html)


def pandoc(src, template, title):
    with open(os.path.join(ROOT, ".toc.tmp"), "w") as f:
        f.write(template)
    try:
        return subprocess.run(
            ["pandoc", "-f", "gfm", "-t", "html5", "--toc", "--toc-depth=2", "--template", os.path.join(ROOT, ".toc.tmp"),
             "-M", "pagetitle=" + title, os.path.join(ROOT, src)], check=True, capture_output=True, text=True).stdout
    finally:
        os.remove(os.path.join(ROOT, ".toc.tmp"))


def render(src, out, title):
    base = os.path.dirname(src)
    body = links(pandoc(src, "$body$", title), base)
    toc = links(pandoc(src, "$toc$", title), base)
    # the Markdown's own centred logo paragraph duplicates the masthead
    body = re.sub(r'<p align="center"><img [^>]*octoburn-256[^>]*></p>\s*', "", body)
    page = open(os.path.join(SITE, "doc_template.html")).read()
    page = page.replace("$pagetitle$", title).replace("$source$", src).replace("$toc$", toc).replace("$body$", body)
    with open(os.path.join(OUT, out), "w") as f:
        f.write(page)


def main():
    shutil.rmtree(OUT, ignore_errors=True)
    shutil.copytree(SITE, OUT, ignore=shutil.ignore_patterns("build.py", "doc_template.html", "__pycache__"))
    shutil.copytree(os.path.join(ROOT, "docs", "img"), os.path.join(OUT, "img", "guide"))
    render("docs/USER_GUIDE.md", "guide.html", "User guide")
    render("README.md", "reference.html", "Reference")
    open(os.path.join(OUT, ".nojekyll"), "w").close()
    print("built", OUT)


if __name__ == "__main__":
    sys.exit(main())
