#!/bin/bash
# /start_workspace.sh

# Get current user and verify it's husarion
CURRENT_USER=$(whoami)
if [ "$CURRENT_USER" != "husarion" ]; then
  echo "This script must be run as 'husarion'."
  exit 1
fi

# Define paths
SYSTEM_COMPOSE="/home/husarion/compose.yaml"
LOCAL_OVERRIDE="./docker-compose.yml"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Export IDs for shared memory permissions
export DOCKER_UID=$(id -u husarion)
export DOCKER_GID=$(id -g husarion)

echo -e "${YELLOW}[1/2] Building custom image with tf_transformations...${NC}"
# This builds only your service using your Dockerfile
docker compose -f $SYSTEM_COMPOSE -f $LOCAL_OVERRIDE build rmit_aiilworkspace

echo -e "${GREEN}[2/2] Launching workspace bash...${NC}"
# This runs your service using both compose files merged
docker compose -f $SYSTEM_COMPOSE -f $LOCAL_OVERRIDE run rmit_aiilworkspace bash