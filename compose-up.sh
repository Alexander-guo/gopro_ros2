#!/usr/bin/env bash
set -e

ARCH=$(uname -m)
OS=$(uname -s | tr '[:upper:]' '[:lower:]')

if [[ "$OS" == "linux" && "$ARCH" == "x86_64" ]]; then
    ENV_FILE="envs/.env.linux"
elif [[ "$OS" == "darwin" && "$ARCH" == "x86_64" ]]; then
    ENV_FILE="envs/.env.mac"
elif [[ "$OS" == "darwin" && "$ARCH" == "arm64" ]]; then
    ENV_FILE="envs/.env.mac-arm"
elif [[ "$OS" == "mingw"* || "$OS" == "msys"* || "$OS" == "cygwin"* ]]; then
    ENV_FILE="envs/.env.win"
else
    echo "Unknown system: $OS/$ARCH"
    exit 1
fi

export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USER=$USER

if [ ! -d "./dataset" ]; then
  mkdir -p ./dataset
  chown $(id -u):$(id -g) ./dataset
  echo "✅ Created dataset folder owned by $(id -un)"
else
  echo "ℹ️ Dataset folder already exists"
fi

echo "🧩 Using environment file: $ENV_FILE"
docker compose --env-file "$ENV_FILE" build

docker compose up -d

