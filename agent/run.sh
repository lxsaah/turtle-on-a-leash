#!/bin/sh
# Starts the agent on the current mission. Run agent/start-mission.sh first.
# Needs `claude` and `aimdb-mcp` on PATH.
cd "$(dirname "$0")" || exit 1
exec claude -p "$(cat prompt.md)" \
    --strict-mcp-config --mcp-config mcp.json \
    --allowedTools mcp__aimdb__get_record,mcp__aimdb__set_record,mcp__aimdb__list_records
