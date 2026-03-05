#!/bin/bash
#
# publish-npm.sh
#
# Script to test build and publish the npm package.
#
# Usage:
#   ./scripts/publish-npm.sh test     # Test build and do a dry-run
#   ./scripts/publish-npm.sh publish  # Build and publish to npm
#

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get the script directory and project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
NPM_DIR="$ROOT_DIR/npm"

echo -e "${GREEN}=== Nexus Physics NPM Publisher ===${NC}\n"

# Check if wasm-pack is installed
if ! command -v wasm-pack &> /dev/null; then
    echo -e "${RED}Error: wasm-pack is not installed${NC}"
    echo "Install it with: cargo install wasm-pack"
    exit 1
fi

# Step 1: Build the package
echo -e "${YELLOW}Step 1: Building package...${NC}"
node "$SCRIPT_DIR/build-npm.mjs"

if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Build successful${NC}\n"

# Step 2: Validate package contents
echo -e "${YELLOW}Step 2: Validating package contents...${NC}"

# Check if required files exist
REQUIRED_FILES=(
    "$NPM_DIR/dist/web/nexus_physics_wasm.js"
    "$NPM_DIR/dist/web/nexus_physics_wasm.d.ts"
    "$NPM_DIR/dist/web/nexus_physics_wasm_bg.wasm"
    "$NPM_DIR/dist/node/nexus_physics_wasm.js"
    "$NPM_DIR/dist/node/nexus_physics_wasm.d.ts"
    "$NPM_DIR/dist/node/nexus_physics_wasm_bg.wasm"
    "$NPM_DIR/package.json"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        echo -e "${RED}Missing required file: $file${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✓ All required files present${NC}\n"

# Step 3: Show package info
echo -e "${YELLOW}Step 3: Package information${NC}"
cd "$NPM_DIR"
PACKAGE_NAME=$(node -p "require('./package.json').name")
PACKAGE_VERSION=$(node -p "require('./package.json').version")
echo "  Name: $PACKAGE_NAME"
echo "  Version: $PACKAGE_VERSION"
echo ""

# Step 4: Action based on argument
ACTION=${1:-test}

if [ "$ACTION" = "test" ]; then
    echo -e "${YELLOW}Step 4: Running dry-run publish (no actual publish)...${NC}"
    npm publish --dry-run

    if [ $? -eq 0 ]; then
        echo -e "\n${GREEN}✓ Dry-run successful!${NC}"
        echo -e "${YELLOW}Package contents that would be published:${NC}"
        npm pack --dry-run
        echo ""
        echo -e "${GREEN}The package is ready to publish.${NC}"
        echo -e "To publish for real, run: ${YELLOW}./scripts/publish-npm.sh publish${NC}"
    else
        echo -e "${RED}Dry-run failed!${NC}"
        exit 1
    fi

elif [ "$ACTION" = "publish" ]; then
    echo -e "${YELLOW}Step 4: Publishing to npm...${NC}"
    echo -e "${RED}⚠️  This will publish $PACKAGE_NAME@$PACKAGE_VERSION to npm!${NC}"
    read -p "Are you sure you want to continue? (yes/no): " -r
    echo

    if [[ $REPLY =~ ^[Yy][Ee][Ss]$ ]]; then
        # Check if logged in to npm
        if ! npm whoami &> /dev/null; then
            echo -e "${RED}You're not logged in to npm.${NC}"
            echo "Please run: npm login"
            exit 1
        fi

        npm publish --access public

        if [ $? -eq 0 ]; then
            echo -e "\n${GREEN}✓ Successfully published $PACKAGE_NAME@$PACKAGE_VERSION!${NC}"
            echo "View at: https://www.npmjs.com/package/$PACKAGE_NAME"
        else
            echo -e "${RED}Publish failed!${NC}"
            exit 1
        fi
    else
        echo "Publish cancelled."
        exit 0
    fi

else
    echo -e "${RED}Invalid argument: $ACTION${NC}"
    echo "Usage: $0 [test|publish]"
    echo "  test    - Build and test with dry-run (default)"
    echo "  publish - Build and publish to npm"
    exit 1
fi

cd "$ROOT_DIR"
