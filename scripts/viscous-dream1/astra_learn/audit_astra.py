import json, sys, re, collections
tools = collections.Counter(); paths = collections.Counter(); robot = collections.Counter(); other = collections.Counter(); imgs = 0
for l in open(sys.argv[1]):
    d = json.loads(l); p = d.get("payload", {})
    if p.get("type") == "custom_tool_call":
        s = p.get("input", "")
        for t in re.findall(r"tools\.([A-Za-z_0-9]+)", s): tools[t] += 1
        for m in re.findall(r"""["'`]((?:/|\.\./|~)[^"'`]*|obs/[^"'`]*|README[^"'`]*)["'`]""", s):
            paths[re.sub(r"\d{5,}", "N", m)] += 1
        for m in re.findall(r"\./robot [a-z]+", s): robot[m] += 1
        for cmd in re.findall(r"""cmd\s*:\s*["'`]([^"'`]*)""", s):
            if not cmd.startswith("./robot"): other[cmd[:100]] += 1
    if p.get("type") == "custom_tool_call_output":
        imgs += sum(1 for o in p.get("output", []) if isinstance(o, dict) and o.get("type") == "input_image")
print("tools", dict(tools)); print("paths", dict(paths)); print("robot", dict(robot)); print("images", imgs)
print("non-robot cmds", other.most_common(20))
