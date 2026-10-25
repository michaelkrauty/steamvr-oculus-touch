# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Release metadata checks; also work in a source archive without Git or third-party modules."""

import datetime
import json
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read(name):
    return (ROOT / name).read_text(encoding="utf-8")


def capture(pattern, name):
    match = re.search(pattern, read(name), re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing release metadata in {name}: {pattern}")
    return match.group(1)


class ReleaseMetadataTests(unittest.TestCase):
    def setUp(self):
        self.version = capture(r'^version = "([0-9]+\.[0-9]+\.[0-9]+)"$', "Cargo.toml")
        self.tag = "v" + self.version
        self.url = "https://github.com/michaelkrauty/steamvr-oculus-touch/releases/tag/" + self.tag

    def test_build_and_citation_versions_agree(self):
        versions = {
            "Cargo.lock": capture(r'^name = "cv1touch"\nversion = "([^"]+)"$', "Cargo.lock"),
            "meson.build": capture(r"^  version: '([^']+)',", "meson.build"),
            "vcpkg.json": json.loads(read("vcpkg.json"))["version-string"],
            "CITATION.cff": capture(r'^version: "([^"]+)"$', "CITATION.cff"),
        }
        for name, version in versions.items():
            with self.subTest(file=name):
                self.assertEqual(version, self.version)
        self.assertEqual(capture(r'^url: "([^"]+)"$', "CITATION.cff"), self.url)

    def test_changelog_has_the_cited_release_and_date(self):
        date = capture(r'^date-released: "([^"]+)"$', "CITATION.cff")
        datetime.date.fromisoformat(date)
        heading = f"## [{self.version}]({self.url}) - {date}"
        self.assertIn(heading, read("CHANGELOG.md").splitlines())
        self.assertEqual(capture(r"^## \[([^]]+)\]", "CHANGELOG.md"), self.version)

    def test_install_instructions_target_this_release(self):
        self.assertIn(f"git clone --branch {self.tag} --depth 1", read("README.md"))
        self.assertIn(f"cv1touch-setup-{self.version}.exe", read("README.md"))
        self.assertIn(f"git checkout --detach {self.tag}", read("docs/setup.md"))
        for name in ["README.md", "docs/setup.md"]:
            with self.subTest(file=name):
                self.assertIn(self.url, read(name))
        for name in ["docs/building-windows.md", "windows/installer/cv1touch.iss"]:
            with self.subTest(file=name):
                self.assertIn(f"/DVersion={self.version}", read(name))


if __name__ == "__main__":
    unittest.main()
