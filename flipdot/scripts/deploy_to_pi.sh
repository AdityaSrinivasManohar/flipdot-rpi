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
SYSTEMD_LOCAL_DIR="/workspaces/flipdot-rpi/flipdot/systemd"

# List of tarballs to scp
TARBALLS=(
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/flipdot_controller_package.tar"
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/flipdot_package.tar"
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/ros2_package.tar"
    "/workspaces/flipdot-rpi/bazel-bin/flipdot/src/foxglove_bridge_package.tar"
)

# --- Argument Parsing ---
# Default to true for both if no args provided
DEPLOY_PACKAGES=true
DEPLOY_SYSTEMD=true
CLEAN_ONLY=false

case "$1" in
    "--systemd")
        DEPLOY_PACKAGES=false
        DEPLOY_SYSTEMD=true
        echo -e "${YELLOW}Mode: Systemd services only.${NC}"
        ;;
    "--packages")
        DEPLOY_PACKAGES=true
        DEPLOY_SYSTEMD=false
        echo -e "${YELLOW}Mode: Packages only.${NC}"
        ;;
    "--clean")
        CLEAN_ONLY=true
        DEPLOY_PACKAGES=false
        DEPLOY_SYSTEMD=false
        echo -e "${RED}Mode: Full Systemd Cleanup.${NC}"
        ;;
    *)
        echo -e "${BLUE}Mode: Full Deployment (Packages + Systemd).${NC}"
        ;;
esac

# --- 1. Cleanup Block ---
if [ "$CLEAN_ONLY" = true ]; then
    echo -e "${RED}⚠️  Cleaning all systemd service files from $PI_HOST...${NC}"
    ssh -t "$PI_USER@$PI_HOST" "
        echo -e '${YELLOW}Stopping and disabling services...${NC}'
        sudo systemctl stop flipdot-rpi.target 2>/dev/null || true
        sudo systemctl disable flipdot-rpi.target 2>/dev/null || true
        sudo systemctl disable 'flipdot-*' 2>/dev/null || true
        sudo systemctl disable 'flipdot_*' 2>/dev/null || true
        
        echo -e '${YELLOW}Removing files from /etc/systemd/system/...${NC}'
        sudo rm -f /etc/systemd/system/flipdot-*
        sudo rm -f /etc/systemd/system/flipdot_*
        sudo rm -f /etc/systemd/system/foxglove-bridge.service
        
        sudo systemctl daemon-reload
        sudo systemctl reset-failed
        echo -e '${GREEN}✅ Systemd files cleared!${NC}'
    "
    exit 0
fi

# --- 2. Package Deployment Block ---
if [ "$DEPLOY_PACKAGES" = true ]; then
    echo -e "${BLUE}🚀 Starting Package Deployment to $PI_HOST...${NC}"

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

        if command -v md5sum >/dev/null 2>&1; then
            LOCAL_MD5=$(md5sum "$TAR" | cut -d' ' -f1)
        else
            LOCAL_MD5=$(md5 -q "$TAR")
        fi

        REMOTE_CHECK=$(ssh "$PI_USER@$PI_HOST" "if [ -f '$TARGET_DIR/$FILENAME' ]; then md5sum '$TARGET_DIR/$FILENAME' | cut -d' ' -f1; else echo 'MISSING'; fi")

        if [ "$LOCAL_MD5" == "$REMOTE_CHECK" ]; then
            echo -e "  ${NC}✨ MD5 match ($LOCAL_MD5). ${GREEN}Skipping upload.${NC}"
            continue
        fi

        echo -e "  ${YELLOW}🔄 Change detected. Updating...${NC}"
        echo -e "  ${RED}🧹 Cleaning remote folder: $TARGET_DIR${NC}"
        ssh "$PI_USER@$PI_HOST" "rm -rf '$TARGET_DIR' && mkdir -p '$TARGET_DIR'"
        
        echo -e "  ${YELLOW}→ Uploading $FILENAME...${NC}"
        scp "$TAR" "$PI_USER@$PI_HOST:$TARGET_DIR/"

        echo -e "  ${YELLOW}→ Extracting...${NC}"
        ssh "$PI_USER@$PI_HOST" "cd $TARGET_DIR && tar -xf $FILENAME"
        echo -e "  ${GREEN}✅ Updated $FOLDER_NAME${NC}"
    done
fi

# --- 3. Systemd Deployment Block ---
if [ "$DEPLOY_SYSTEMD" = true ]; then
    echo -e "${BLUE}==================================================${NC}"
    echo -e "${BLUE}⚙️  Deploying Systemd Services...${NC}"

    ssh "$PI_USER@$PI_HOST" "mkdir -p ~/systemd_tmp"
    scp $SYSTEMD_LOCAL_DIR/* "$PI_USER@$PI_HOST:~/systemd_tmp/"

    echo -e "${YELLOW}🔄 Registering services and reloading systemd...${NC}"
    ssh -t "$PI_USER@$PI_HOST" "
        sudo mv ~/systemd_tmp/* /etc/systemd/system/
        sudo rm -rf ~/systemd_tmp
        sudo systemctl daemon-reload
        sudo systemctl enable flipdot-rpi.target
        echo -e '${GREEN}✅ Services registered successfully!${NC}'
        
        echo -e '${BLUE}📊 Current Dependency Tree for flipdot-rpi.target:${NC}'
        systemctl list-dependencies flipdot-rpi.target
    "
fi

echo -e "${BLUE}--------------------------------------------------${NC}"
echo -e "${GREEN}✨ Sync complete!${NC}"