#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

required=(
  README.md
  AGENTS.md
  备赛/PROJECT_CONTEXT.md
  备赛/00_赛事基线.md
  备赛/02_选材与库存盘点.md
  备赛/04_代码复现规范.md
  hardware/bom.csv
  templates/problem/verification.md
  archive/bupt-2026-race-car/ARCHIVE.md
)

for file in "${required[@]}"; do
  if [[ ! -f "$file" ]]; then
    echo "缺少必需文件: $file" >&2
    exit 1
  fi
done

if ! git rev-parse -q --verify refs/tags/archive/bupt-2026-race-car-final-20260717 >/dev/null; then
  echo "缺少旧项目归档标签" >&2
  exit 1
fi

tracked_generated="$(git ls-files \
  | grep -E '(^|/)(Debug|build)/|\.(o|d|out|map)$' \
  | grep -Ev '^problems/[^/]+/release/[^/]+\.(out|map)$' \
  || true)"
if [[ -n "$tracked_generated" ]]; then
  echo "发现不应跟踪的生成文件:" >&2
  echo "$tracked_generated" >&2
  exit 1
fi

echo "仓库结构与归档入口检查通过"
