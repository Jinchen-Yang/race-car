#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "用法: $0 <YYYYMMDD-题目代号>" >&2
  exit 2
fi

slug="$1"
if [[ ! "$slug" =~ ^[0-9]{8}-[a-z0-9][a-z0-9-]*$ ]]; then
  echo "题目代号必须形如 20260724-measurement-a" >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
template="$repo_root/templates/problem"
destination_root="${PROBLEM_ROOT:-$repo_root/problems}"
destination="$destination_root/$slug"

if [[ -e "$destination" ]]; then
  echo "目标已存在: $destination" >&2
  exit 1
fi

mkdir -p "$destination"/{firmware,hardware,evidence,release}
cp "$template/README.md" "$destination/README.md"
cp "$template/requirements.md" "$destination/requirements.md"
cp "$template/verification.md" "$destination/verification.md"
cp "$template/bom.csv" "$destination/bom.csv"

echo "已创建: $destination"
