#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <git-repository-url>" >&2
  exit 1
fi

repo_url="$1"

git init
git add README.md docs config interfaces src media \
  .gitignore .env.example requirements.txt \
  THIRD_PARTY_AND_LICENSE_NOTES.md upload_to_github.sh

git status
echo
echo "Review the staged files above before committing."
read -r -p "Commit and push these files? [y/N] " answer
if [[ ! "$answer" =~ ^[Yy]$ ]]; then
  echo "Stopped before commit."
  exit 0
fi

git commit -m "Document ROBOTEL vision and robot control contribution"
git branch -M main
git remote add origin "$repo_url"
git push -u origin main
