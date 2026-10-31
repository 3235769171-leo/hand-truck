"""Compile the unmodified driver source against host hardware shims and run it."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = shutil.which("gcc")
if not compiler:
    raise SystemExit("GCC is required to run the JC014 host tests")

with tempfile.TemporaryDirectory(prefix="jc014-tests-") as output_dir:
    binary = Path(output_dir) / ("jc014_tests.exe" if os.name == "nt" else "jc014_tests")
    command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
               "-include", str(root / "tests/jc014_host_platform.h"),
               "-I" + str(root / "Core/Inc"),
               "-I" + str(root / "Middlewares/Third_Party/FreeRTOS/Source/include"),
               str(root / "Core/Src/jc014.c"),
               str(root / "tests/jc014_host_test.c"), "-o", str(binary)]
    subprocess.run(command, check=True, cwd=str(root))
    raise SystemExit(subprocess.call([str(binary)], cwd=str(root)))
