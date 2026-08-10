#!/usr/bin/env bash
set -euo pipefail

repo_name="smart_safety_band"
description="ESP-IDF firmware for SIM868 safety band with dynamic NVS emergency routing"
topics=(esp-idf esp32s3 sim868 safety-band nvs)

cd "$(git rev-parse --show-toplevel)"

if ! command -v gh >/dev/null 2>&1; then
  echo "ERROR: GitHub CLI (gh) is not installed. Install it and retry."
  exit 1
fi

if ! gh auth status >/dev/null 2>&1; then
  echo "ERROR: gh is not authenticated. Run 'gh auth login' or 'gh auth login --with-token < /path/to/token' first."
  exit 1
fi

# Ensure the local branch is main.
git branch -M main

git add .github/workflows/ci.yml
if git diff --cached --quiet; then
  echo "No new staged changes to commit for CI workflow."
else
  git commit -m "Add GitHub Actions CI workflow for ESP-IDF build"
fi

# Create repository if it does not already exist.
if ! gh repo view "$(git remote get-url origin 2>/dev/null || true)" >/dev/null 2>&1; then
  gh repo create "$repo_name" --public --source . --remote origin --push --description "$description"
else
  echo "Repository already exists or origin is already configured. Skipping repo create."
fi

gh repo edit --description "$description"
gh repo edit --add-topic "${topics[@]}"

echo "GitHub repository setup complete."
