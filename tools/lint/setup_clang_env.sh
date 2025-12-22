#!/usr/bin/env bash
set -euo pipefail

# --- Configuration ---
WORKSPACE_ROOT="/workspaces/flipdot-rpi"
cd "$WORKSPACE_ROOT"
LLVM_TOOLCHAIN_PATH="$(bazel info output_base)/external/toolchains_llvm++llvm+llvm_toolchain_21"
TARGET_CLANGD_LINK="$WORKSPACE_ROOT/tools/lint/clangd"
TARGET_CLANG_TIDY_LINK="$WORKSPACE_ROOT/.clang-tidy"
SOURCE_CLANG_TIDY="$WORKSPACE_ROOT/tools/lint/.clang-tidy"
TARGET_CLANG_FORMAT_LINK="$WORKSPACE_ROOT/.clang-format"
SOURCE_CLANG_FORMAT="$WORKSPACE_ROOT/tools/lint/.clang-format"

echo "Setting up Clang environment for VSCode..."
echo "Workspace: $WORKSPACE_ROOT"

# --- Check that Bazel has fetched the toolchain ---
if [ ! -d "$LLVM_TOOLCHAIN_PATH" ]; then
  echo "LLVM toolchain not found in Bazel cache. Running a fetch..."
  bazel fetch @llvm_toolchain_21//:clangd >/dev/null
fi

# --- Find clangd binary ---
CLANGD_PATH="$(find "$LLVM_TOOLCHAIN_PATH" -name clangd | head -n 1 || true)"

if [ -z "$CLANGD_PATH" ]; then
  echo "❌ Could not locate clangd inside $LLVM_TOOLCHAIN_PATH"
  exit 1
fi

# --- Create symlink for clangd ---
mkdir -p "$(dirname "$TARGET_CLANGD_LINK")"
ln -sf "$CLANGD_PATH" "$TARGET_CLANGD_LINK"
echo "✅ Linked clangd -> $CLANGD_PATH"

# --- Symlink .clang-tidy config ---
if [ -f "$SOURCE_CLANG_TIDY" ]; then
  ln -sf "$SOURCE_CLANG_TIDY" "$TARGET_CLANG_TIDY_LINK"
  echo "✅ Linked .clang-tidy -> $SOURCE_CLANG_TIDY"
else
  echo "⚠️ No .clang-tidy found in tools/lint/, skipping."
fi

# --- Symlink .clang-format config ---
if [ -f "$SOURCE_CLANG_FORMAT" ]; then
  ln -sf "$SOURCE_CLANG_FORMAT" "$TARGET_CLANG_FORMAT_LINK"
  echo "✅ Linked .clang-format -> $SOURCE_CLANG_FORMAT"
else
  echo "⚠️ No .clang-format found in tools/lint/, skipping."
fi

echo "🎉 Setup complete. You can now point VSCode to: tools/lint/clangd"
