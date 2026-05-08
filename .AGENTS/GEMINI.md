# gstack

> **[ 가장 우선되는 전역 시스템 규칙 ]**
> 모든 AI 에이전트는 대화 시작, 새로운 스킬 사용(특히 `/review`, `/qa`, 코딩 지원 작동) 시 **반드시 `AGENTS.md` 파일을 가장 먼저 읽고**, 해당 파일에 정의된 모든 설계/안전 정책(Hardware Isolation, RTOS 메모리 제약, Failsafe 로직 등)을 절대적인 잣대로 반영하여 응답해야 합니다.

Use the `/browse` skill from gstack for all web browsing. NEVER use `mcp__anitigravity-in-chrome__*` tools.

## Available Skills
- /office-hours
- /plan-ceo-review
- /plan-eng-review
- /plan-design-review
- /design-consultation
- /review
- /ship
- /land-and-deploy
- /canary
- /benchmark
- /browse
- /qa
- /qa-only
- /design-review
- /setup-browser-cookies
- /setup-deploy
- /retro
- /investigate
- /document-release
- /codex
- /cso
- /autoplan
- /careful
- /freeze
- /guard
- /unfreeze
- /gstack-upgrade
