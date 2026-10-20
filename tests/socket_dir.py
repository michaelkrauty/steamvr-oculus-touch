# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 michaelkrauty
"""Private Linux fixtures with short socket names, independent of TMPDIR length."""
import os
from pathlib import Path
import tempfile


class SocketDirectory:
    def __init__(self):
        self._temporary = tempfile.TemporaryDirectory(prefix="cv1t-s-")
        self._descriptor = None
        try:
            os.chmod(self._temporary.name, 0o700)
            self._descriptor = os.open(self._temporary.name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            # Keep the owner's descriptor open through subprocess clients. A real child
            # preserves final-component symlink checks; no file is created outside this tree.
            self.name = f"/proc/{os.getpid()}/fd/{self._descriptor}/r"
            Path(self.name).mkdir(mode=0o700)
            os.chmod(self.name, 0o700)
        except BaseException:
            self.cleanup()
            raise

    def cleanup(self):
        try:
            self._temporary.cleanup()
        finally:
            if self._descriptor is not None:
                os.close(self._descriptor)
                self._descriptor = None
