#!/bin/sh
# SPDX-License-Identifier: MIT
# Fails when a source file lacks an SPDX-License-Identifier line in its first five lines.
# Checked: C/C++ sources and headers, CMake files, shell scripts, YAML, python/**/*.py, pyproject.toml. Skipped: third_party/, build directories,
# generated documentation, golden data. Usage: tools/check-spdx.sh [root]   (default: the repository root)
set -eu
root=${1:-$(cd "$(dirname "$0")/.." && pwd)}
cd "$root"
missing=0
list=$(if git rev-parse --git-dir > /dev/null 2>&1; then git ls-files; else find . -type f | sed 's|^\./||'; fi)
for f in $(printf '%s\n' "$list" | grep -E '\.(h|hpp|cpp|c|cmake|sh|yml|yaml|in)$|^python/.*\.py$|(^|/)pyproject\.toml$|(^|/)CMakeLists\.txt$' \
        | grep -v -E '^(third_party|build[^/]*)/' | grep -v -E '/build[^/]*/' ); do
    [ -f "$f" ] || continue
    if ! head -n 5 "$f" | grep -q 'SPDX-License-Identifier:'; then
        echo "missing SPDX header: $f"
        missing=$((missing + 1))
    fi
done
if [ "$missing" -ne 0 ]; then
    echo "$missing file(s) without an SPDX-License-Identifier line"
    exit 1
fi
echo "SPDX headers present"
