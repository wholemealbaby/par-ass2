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

# Container name (must match docker-compose.yml container_name)
CONTAINER_NAME="g30_snc"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Export IDs for shared memory permissions
export DOCKER_UID=$(id -u husarion)
export DOCKER_GID=$(id -g husarion)

echo -e "${YELLOW}[1/3] Checking for existing container '${CONTAINER_NAME}'...${NC}"

# Check if container exists
if docker container ls -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo -e "${YELLOW}Container '${CONTAINER_NAME}' found.${NC}"
  
  # Check if container is running
  if docker container ls --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${GREEN}Container '${CONTAINER_NAME}' is already running.${NC}"
    echo -e "${YELLOW}Reconnecting to existing container...${NC}"
    docker exec -it ${CONTAINER_NAME} bash
    exit 0
  else
    echo -e "${YELLOW}Container exists but is stopped. Starting container...${NC}"
    docker start ${CONTAINER_NAME} > /dev/null
    echo -e "${GREEN}Container started. Reconnecting...${NC}"
    docker exec -it ${CONTAINER_NAME} bash
    exit 0
  fi
else
  echo -e "${YELLOW}Container '${CONTAINER_NAME}' not found. Proceeding with build and run.${NC}"
fi

echo -e "${YELLOW}[2/3] Building custom image with tf_transformations...${NC}"
# This builds only your service using your Dockerfile
docker compose -f $SYSTEM_COMPOSE -f $LOCAL_OVERRIDE build rmit_aiilworkspace

echo -e "${GREEN}[3/3] Launching workspace bash...${NC}"
# This runs your service using both compose files merged
docker compose -f $SYSTEM_COMPOSE -f $LOCAL_OVERRIDE run rmit_aiilworkspace bash
