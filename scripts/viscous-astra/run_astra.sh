#!/usr/bin/env bash
# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Launch the bounded operator loop adapted from metal-arm-harness OPERATING.md.
set -euo pipefail
if (( $# != 2 )); then
    echo 'Usage: run_astra.sh SESSION_DIR "task for Astra"' >&2
    exit 2
fi
here=$(cd -- "$(dirname -- "$0")" && pwd -P)
session=$(cd -- "$1" && pwd -P)
cd -- "$session/agent"
py=$(command -v python3)
# Codex reads its MCP command/args as TOML; encode literal paths including spaces.
mcp_args=$("$py" -Ic 'import json,sys;print(json.dumps(sys.argv[1:]))' -I "$here/agent_bridge.py" "$session")
backend=$("$py" -Ic 'import json,sys;print(json.load(open(sys.argv[1]))["backend"])' "$session/connection.json")
policy=(--disable shell_tool --disable unified_exec)
wrapper=(env)
if [[ ${ASTRA_FAKE_SANDBOX_PROBE:-0} == 1 ]]; then
    [[ $backend == fake && $(uname -s) == Linux ]] || exit 2
    policy=()
elif [[ $backend == lab ]]; then
    [[ $(uname -s) == Darwin ]] || exit 2
    # A second OS boundary around Codex and every descendant, including configured tools.
    # No serial, raw disk, video-device or IOKit user-client opens, even outside tool sandboxes.
    # Keep local Lab HTTP and other loopback services outside the agent's reach.
    # The trusted bridge may connect to the one bound TCP port if a TCP session was selected.
    tcp_rule=$("$py" -Ic 'import json,sys; e=json.load(open(sys.argv[1])); print("(allow network-outbound (remote tcp \"localhost:%d\"))"%int(e["tcp"].split(":")[1]) if "tcp" in e else "")' "$session/connection.json")
    wrapper=(/usr/bin/sandbox-exec -p '(version 1)(allow default)(deny iokit-open)(deny appleevent-send)(deny file-read* file-write* (regex #"^/dev/(cu[.]|tty[.]|video|disk|rdisk)"))(deny network-outbound (remote ip "localhost:*"))'"$tcp_rule")
fi
# Close out a model error or an early final answer through the protocol, never a signal.
cleanup() {
    if [[ ! -f "$session/stop.json" ]]; then
        "$py" "$here/astra_op.py" op --session "$session" quit >> "$session/launcher-stop.log" 2>&1 || true
    fi
}
trap cleanup EXIT
{
    cat "$here/OPERATING.md"
    printf '\nSession task\n%s\n' "$2"
} | "$py" -I "$here/agent_run.py" "$session" "${wrapper[@]}" codex exec --ignore-user-config --ignore-rules --skip-git-repo-check --ephemeral \
    --disable apps --disable plugins --disable multi_agent --disable image_generation "${policy[@]}" \
    -c 'web_search="disabled"' -c 'approval_policy="never"' \
    -c "mcp_servers.viscous.command=\"$py\"" -c "mcp_servers.viscous.args=$mcp_args" \
    -c 'mcp_servers.viscous.enabled_tools=["op","frames"]' -c 'mcp_servers.viscous.default_tools_approval_mode="approve"' \
    -c 'mcp_servers.viscous.required=true' -c 'mcp_servers.viscous.tool_timeout_sec=600' \
    -m gpt-6.2-astra -c 'model_reasoning_effort="high"' -s workspace-write \
    -C "$session/agent" --json -o "$session/astra-final.txt" -
cat "$session/astra-final.txt"
