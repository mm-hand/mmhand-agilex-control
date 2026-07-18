# third_party

Vendor dependencies that should stay outside the main source tree live here.

`pyAgxArm` is referenced as a git submodule:

```text
third_party/pyAgxArm/
```

After cloning this repository, initialize it with:

```bash
git submodule update --init --recursive
```

`hardware.py` adds `third_party/pyAgxArm` to `sys.path` at runtime.
