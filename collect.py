"""
Data collection task runner.
"""

import sys
from pathlib import Path

try:
    import depthai
except ImportError:
    depthai_repo_base = Path.home() / "workspace" / "depthai"
    depthai_sdk_base = depthai_repo_base / "depthai_sdk" / "src"
    sys.path.append(depthai_repo_base.absolute())
    sys.path.append(depthai_sdk_base.absolute())
    print(f"Using depthai from {depthai_repo_base}")

from sensors import oak_d

oak_d.collect(resolution="800")