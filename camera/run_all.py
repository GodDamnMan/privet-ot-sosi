import subprocess
from pathlib import Path
import sys

mode = sys.argv[1] if len(sys.argv) > 1 else "solution.py"
tests_dir = Path("tests")
ok = bad = 0

for test_file in sorted(f for f in tests_dir.iterdir() if not f.suffix):
    clue_file = Path(str(test_file) + ".clue")
    if not clue_file.exists():
        continue

    with open(test_file, "rb") as f:
        proc = subprocess.run(["python3", mode], stdin=f, capture_output=True, text=True)

    got = proc.stdout.strip()
    expected = clue_file.read_text().strip()

    if got == expected:
        print(f"OK  {test_file.name}  -> {got}")
        ok += 1
    else:
        print(f"BAD {test_file.name}  got: {got}  expected: {expected}")
        bad += 1

print(f"\nOK: {ok}  BAD: {bad}")