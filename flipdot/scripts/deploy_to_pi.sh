#!/bin/bash
set -e

# --- Colors ---
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# --- Configuration ---
PI_USER="adsm"
PI_HOST="10.0.0.192"
PI_BASE_DIR="/home/adsm/flipdot"

# List of tarballs to scp
TARBALLS=(
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/flipdot_controller_package.tar"
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/flipdot_package.tar"
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/ros2_package.tar"
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/foxglove_bridge_package.tar"
)

echo -e "${BLUE}🚀 Starting Deployment to $PI_HOST...${NC}"

for TAR in "${TARBALLS[@]}"; do
    if [ ! -f "$TAR" ]; then
        echo -e "${RED}❌ Local file missing: $TAR${NC}"
        exit 1
    fi

    FILENAME=$(basename "$TAR")
    FOLDER_NAME="${FILENAME%.tar}"
    TARGET_DIR="$PI_BASE_DIR/$FOLDER_NAME"

    echo -e "${BLUE}--------------------------------------------------${NC}"
    echo -e "${GREEN}📦 Package:${NC} $FOLDER_NAME"

    # Mac 'md5' vs Linux 'md5sum' check
    if command -v md5sum >/dev/null 2>&1; then
        LOCAL_MD5=$(md5sum "$TAR" | cut -d' ' -f1)
    else
        LOCAL_MD5=$(md5 -q "$TAR")
    fi

    # We use a single SSH call to check both existence and the checksum to save time
    REMOTE_CHECK=$(ssh "$PI_USER@$PI_HOST" "if [ -f '$TARGET_DIR/$FILENAME' ]; then md5sum '$TARGET_DIR/$FILENAME' | cut -d' ' -f1; else echo 'MISSING'; fi")

    if [ "$LOCAL_MD5" == "$REMOTE_CHECK" ]; then
        echo -e "  ${NC}✨ MD5 match ($LOCAL_MD5). ${GREEN}Skipping upload and extraction.${NC}"
        continue
    fi

    # Proceed with update if MD5 differs or file is missing
    echo -e "  ${YELLOW}🔄 Change detected or file missing. Updating...${NC}"

    # Remove the old folder contents before proceeding
    echo -e "  ${RED}🧹 Cleaning remote folder: $TARGET_DIR${NC}"
    ssh "$PI_USER@$PI_HOST" "rm -rf '$TARGET_DIR' && mkdir -p '$TARGET_DIR'"
    
    ssh "$PI_USER@$PI_HOST" "mkdir -p $TARGET_DIR"
    
    echo -e "  ${YELLOW}→ Uploading $FILENAME...${NC}"
    scp "$TAR" "$PI_USER@$PI_HOST:$TARGET_DIR/"

    echo -e "  ${YELLOW}→ Extracting...${NC}"
    ssh "$PI_USER@$PI_HOST" "cd $TARGET_DIR && tar -xf $FILENAME"
    
    echo -e "  ${GREEN}✅ Successfully updated $FOLDER_NAME${NC}"
done

echo -e "${BLUE}--------------------------------------------------${NC}"
echo -e "${GREEN}✨ Deployment sync complete!${NC}"
