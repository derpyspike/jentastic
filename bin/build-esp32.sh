#!/usr/bin/env bash

set -e

VERSION=`bin/buildinfo.py long`
SHORT_VERSION=`bin/buildinfo.py short`

BUILDDIR=.pio/build/$1
OUTDIR=release

rm -f $OUTDIR/firmware*
rm -r $OUTDIR/* || true

# Important to pull latest version of libs into all device flavors, otherwise some devices might be stale
# platformio pkg install -e $1
# ...redundant with pioarduino

echo "Building for $1 with $PLATFORMIO_BUILD_FLAGS"
rm -f $BUILDDIR/firmware*

# Guard against PlatformIO 6.2.0 auto-upgrade and gutted SCons Fortran bug
mkdir -p "$HOME/.platformio"
if [ ! -f "$HOME/.platformio/appstate.json" ]; then
  echo '{"last_version": "6.1.19", "last_check": {"platformio_upgrade": 2147483647, "prune_system": 2147483647}}' > "$HOME/.platformio/appstate.json"
fi

# Ensure FortranCommon fallback exists in any installed SCons Tool directory
for scons_tool_dir in $(find "$HOME/.platformio/packages" -maxdepth 3 -type d -name "Tool" 2>/dev/null); do
  if [ -f "$scons_tool_dir/linkCommon/__init__.py" ] && [ ! -f "$scons_tool_dir/FortranCommon.py" ]; then
    printf '%s\n' \
      '"""Fallback shim for SCons FortranCommon to prevent link crashes"""' \
      'def isfortran(env, source): return False' \
      'def add_all_to_env(env): pass' \
      'def add_fortran_to_env(env): pass' > "$scons_tool_dir/FortranCommon.py"
  fi
done

# The shell vars the build tool expects to find
export APP_VERSION=$VERSION

basename=firmware-$1-$VERSION

pio run --environment $1 -t mtjson # -v

cp $BUILDDIR/$basename.elf $OUTDIR/$basename.elf

echo "Copying ESP32 bin file"
cp $BUILDDIR/$basename.factory.bin $OUTDIR/$basename.factory.bin

echo "Copying ESP32 update bin file"
cp $BUILDDIR/$basename.bin $OUTDIR/$basename.bin

echo "Copying Filesystem for ESP32 targets"
cp $BUILDDIR/littlefs-$1-$VERSION.bin $OUTDIR/littlefs-$1-$VERSION.bin
cp bin/device-install.* $OUTDIR/
cp bin/device-update.* $OUTDIR/

echo "Copying manifest"
cp $BUILDDIR/$basename.mt.json $OUTDIR/$basename.mt.json
