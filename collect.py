"""
Data collection task runner.
"""

import sys
import os
from pathlib import Path
from datetime import datetime

try:
    import depthai
except ImportError:
    depthai_repo_base = Path.home() / "workspace" / "depthai"
    depthai_sdk_base = depthai_repo_base / "depthai_sdk" / "src"
    sys.path.append(depthai_repo_base.absolute())
    sys.path.append(depthai_sdk_base.absolute())
    print(f"Using depthai from {depthai_repo_base}")

from sensors import oak_d

timestamp = datetime.now().isoformat().replace(':', '-').replace('.', '_')
folder = Path.home() / "Captures" / "OAK-D" / timestamp
os.makedirs(folder.absolute(), exist_ok=False)

oak_d.collect(folder, resolution="800")
