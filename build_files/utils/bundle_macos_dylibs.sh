#!/usr/bin/env bash
# Fix macOS .app bundle to be fully self-contained.
# Finds all dylibs referenced from outside the bundle (e.g. Homebrew)
# and copies them in, rewriting load paths to @rpath.
#
# Usage: ./bundle_macos_dylibs.sh /path/to/Blender-SDF.app

set -euo pipefail

APP="$1"
if [[ ! -d "$APP" ]]; then
  echo "Usage: $0 /path/to/Blender-SDF.app"
  exit 1
fi

MACOS_DIR="$APP/Contents/MacOS"
LIB_DIR="$APP/Contents/Resources/lib"
mkdir -p "$LIB_DIR"

BINARY="$MACOS_DIR/Blender"
if [[ ! -f "$BINARY" ]]; then
  echo "Error: $BINARY not found"
  exit 1
fi

fix_binary() {
  local bin="$1"
  local deps
  deps=$(otool -L "$bin" 2>/dev/null | tail -n +2 | awk '{print $1}')

  for dep in $deps; do
    # Skip system frameworks/libs and already-fixed paths
    case "$dep" in
      /usr/lib/*|/System/*|@rpath/*|@loader_path/*|@executable_path/*)
        continue
        ;;
    esac

    local basename
    basename=$(basename "$dep")
    local dest="$LIB_DIR/$basename"

    if [[ ! -f "$dest" ]]; then
      echo "  Bundling: $dep -> $dest"
      cp "$dep" "$dest" 2>/dev/null || { echo "  WARNING: Could not copy $dep"; continue; }
      chmod 644 "$dest"
      # Recursively fix the newly copied lib
      fix_binary "$dest"
    fi

    echo "  Rewriting: $dep -> @rpath/$basename in $(basename "$bin")"
    install_name_tool -change "$dep" "@rpath/$basename" "$bin" 2>/dev/null || true
  done
}

echo "Fixing main binary..."
fix_binary "$BINARY"

echo "Fixing bundled dylibs..."
for dylib in "$LIB_DIR"/*.dylib; do
  [[ -f "$dylib" ]] || continue
  fix_binary "$dylib"
  # Set the dylib's own id to @rpath-relative
  local_name=$(basename "$dylib")
  install_name_tool -id "@rpath/$local_name" "$dylib" 2>/dev/null || true
done

echo "Verifying no external dependencies remain..."
LEAKS=0
for f in "$BINARY" "$LIB_DIR"/*.dylib; do
  [[ -f "$f" ]] || continue
  external=$(otool -L "$f" 2>/dev/null | tail -n +2 | awk '{print $1}' | grep -v -E '^(/usr/lib/|/System/|@rpath/|@loader_path/|@executable_path/)' || true)
  if [[ -n "$external" ]]; then
    echo "  STILL EXTERNAL in $(basename "$f"):"
    echo "$external" | sed 's/^/    /'
    LEAKS=1
  fi
done

if [[ $LEAKS -eq 0 ]]; then
  echo "OK: Bundle is fully self-contained."
else
  echo "WARNING: Some external dependencies remain. Re-run or fix manually."
fi

echo ""
echo "To re-sign (required after modifying binaries):"
echo "  codesign --force --deep --sign - $APP"
