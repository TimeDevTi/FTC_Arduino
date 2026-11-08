#!/usr/bin/env python3
"""Build and run the platform-independent drive-mixing/pin-map tests."""

from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="mecanum-native-") as temporary:
        executable = Path(temporary) / "test_mecanum"
        command = [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{ROOT / 'include'}",
            str(ROOT / "test/native/test_mecanum.cpp"),
            "-o",
            str(executable),
        ]
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run([str(executable)], cwd=ROOT, check=True)
    print("Native mecanum/pin-map tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
