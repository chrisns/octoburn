# coding=utf-8
"""Static guards on the view model JS, which has no test harness of its own.

2026-11-16: `self.tapTest` was first a ko.observable (the tap-test setting) and then
reassigned to the button's function. Loading the settings called `self.tapTest(value)`, which
STARTED a tap test on every status refresh, so an unreachable ESP produced a stream of failed
runs. A name may be an observable or a function, never both.
"""
import os
import re

JS = os.path.join(os.path.dirname(__file__), os.pardir, "octoprint_laserheightprobe", "static",
                  "js", "laserheightprobe.js")


def _source():
    with open(JS) as handle:
        return handle.read()


def test_no_name_is_both_an_observable_and_a_function():
    source = _source()
    observables = set(re.findall(r"self\.(\w+)\s*=\s*ko\.(?:observable|observableArray|pureComputed|computed)\b",
                                 source))
    functions = set(re.findall(r"self\.(\w+)\s*=\s*function\b", source))
    assert observables & functions == set()


def test_no_view_model_member_is_assigned_twice_at_construction():
    # Members created with ko.* or function must be created once; plain values may change.
    source = _source()
    names = re.findall(r"self\.(\w+)\s*=\s*(?:ko\.\w+|function)\b", source)
    duplicates = sorted(set(n for n in names if names.count(n) > 1))
    assert duplicates == []
