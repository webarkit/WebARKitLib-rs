#!/bin/bash
# coverage.sh — Generate a local HTML code coverage report using cargo-tarpaulin.
#
# Usage:
#   ./scripts/coverage.sh
#
# Requires cargo-tarpaulin to be installed:
#   cargo install cargo-tarpaulin
set -e

echo "🧪 Generating Code Coverage Report..."
echo ""

# Clean previous coverage output
rm -rf coverage/
mkdir -p coverage/

echo "Using Tarpaulin for coverage..."
cargo tarpaulin \
    --workspace \
    --out Html \
    --output-dir coverage \
    --timeout 300 \
    --exclude-files "tests/*" \
    --ignore-panics \
    --ignore-timeouts \
    --verbose

echo ""
echo "✅ Coverage report generated!"
echo ""
echo "📊 Open the report:"
if command -v open &> /dev/null; then
    open coverage/index.html        # macOS
elif command -v xdg-open &> /dev/null; then
    xdg-open coverage/index.html   # Linux
else
    echo "   $(pwd)/coverage/index.html"
fi
