#!/usr/bin/env python3
from pathlib import Path
import runpy


if __name__ == "__main__":
    module_path = Path(__file__).resolve().with_name("inter_drone_collision_attack.py")
    main = runpy.run_path(str(module_path))["main"]
    raise SystemExit(main())
