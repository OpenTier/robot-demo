#!/bin/bash
set -e

PROJECT_ROOT="$(realpath "$(dirname "$0")/..")"

# ROS2 workspace directory
WORKSPACE_DIR="$PROJECT_ROOT/edge_device/ros2_ws"

DOCKERFILE="$PROJECT_ROOT/docker/Dockerfile.bundle"
# Directory where the final archive will be saved
OUTPUT_DIR="bundle_output"
# Name of the output archive
ARCHIVE_NAME="ros2_bundle.tar.gz"
# Target platform for Raspberry Pi
TARGET_PLATFORM="linux/arm64"
# Tag for the Docker image
IMAGE_TAG="ros2_bundle:latest"

# Check that the workspace directory exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace directory '$WORKSPACE_DIR' not found in the current directory!"
    exit 1
fi

echo "Setting up QEMU for ARM emulation..."
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

BUILDER_NAME="multiarch_builder"
if ! docker buildx inspect "$BUILDER_NAME" > /dev/null 2>&1; then
    echo "Creating Docker Buildx builder '$BUILDER_NAME' with increased memory..."
    docker buildx create --name "$BUILDER_NAME" --use --driver docker-container --driver-opt memory=8g
else
    echo "Using existing Docker Buildx builder '$BUILDER_NAME'..."
    docker buildx use "$BUILDER_NAME"
fi
docker buildx inspect --bootstrap

# Clear the OUTPUT_DIR if it already exists
# if [ -d "$OUTPUT_DIR" ]; then
#     echo "Clearing existing output directory '$OUTPUT_DIR'..."
#     rm -rf "$OUTPUT_DIR"
# fi

echo "Building the ARM64 ROS2 bundle..."
mkdir -p $OUTPUT_DIR
docker buildx build --platform $TARGET_PLATFORM -f $DOCKERFILE --output type=local,dest=$OUTPUT_DIR -t $IMAGE_TAG .

echo "Bundle built successfully."
echo "The bundle archive is located at '$OUTPUT_DIR/workspace/ros2_bundle.tar.gz'."
