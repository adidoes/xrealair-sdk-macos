#!/bin/bash
set -e

# Find current version in CMakeLists.txt
CURRENT_VERSION=$(grep -E "VERSION [0-9]+\.[0-9]+\.[0-9]+" CMakeLists.txt | head -1 | awk '{print $2}')

if [ -z "$CURRENT_VERSION" ]; then
    echo "Error: Could not find version in CMakeLists.txt"
    exit 1
fi

echo "Current version: $CURRENT_VERSION"

# Split version into major, minor, patch
IFS='.' read -r -a version_parts <<< "$CURRENT_VERSION"
MAJOR="${version_parts[0]}"
MINOR="${version_parts[1]}"
PATCH="${version_parts[2]}"

# Calculate default next version (patch bump)
DEFAULT_NEXT_VERSION="$MAJOR.$MINOR.$((PATCH + 1))"

# Ask for next version
read -p "Enter next version [$DEFAULT_NEXT_VERSION]: " NEXT_VERSION
NEXT_VERSION=${NEXT_VERSION:-$DEFAULT_NEXT_VERSION}

echo "Updating to version: $NEXT_VERSION"

# Update version in CMakeLists.txt
sed -i '' "s/VERSION $CURRENT_VERSION/VERSION $NEXT_VERSION/g" CMakeLists.txt
sed -i '' "s/SOVERSION $CURRENT_VERSION/SOVERSION $NEXT_VERSION/g" CMakeLists.txt

# Check if the update was successful
if ! grep -q "VERSION $NEXT_VERSION" CMakeLists.txt; then
    echo "Error: Failed to update version in CMakeLists.txt"
    exit 1
fi

# Git commit and tag
git add CMakeLists.txt
git commit -m "bump to version $NEXT_VERSION"
git tag "v$NEXT_VERSION"
git push origin "v$NEXT_VERSION"

echo "Successfully bumped version to $NEXT_VERSION and pushed tag v$NEXT_VERSION"