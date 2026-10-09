# Repository instructions

Maintain a portable, reproducible ROS1 bag and Isaac replay bundle. Chinese: [AGENTS.zh-CN.md](AGENTS.zh-CN.md).

## Rules

- Keep Markdown concise and paired in English and Simplified Chinese.
- Enforce [TABLETOP_FRAME_CONTRACT.md](TABLETOP_FRAME_CONTRACT.md): physical base is exactly0.45m above the tabletop; do not move mounting geometry to pass collision checks.
- Runtime binary files belong to the private GitHub Release. Keep only required current assets; no logs, backups, changelogs or intermediate output directories in Git.
- Use clone-relative paths; verify artifact SHA-256 bindings before release.
- Body joint order is unconfirmed. Keep discrete collision, physical replay, CCD and hardware claims separate.
