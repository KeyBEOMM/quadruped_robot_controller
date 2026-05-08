# Agent Configuration and Instructions

This project utilizes the `gstack` skills ecosystem to enhance AI agent capabilities. Any AI agent operating within this repository MUST follow these instructions.

## Web Browsing Policy
- **ALWAYS** use the `/browse` skill from gstack for all web browsing and research tasks.
- **NEVER** use native browser tools (such as `mcp__claude-in-chrome__*` or default browser integrations).

## Available `gstack` Skills
The following skills are available for use in this repository:
- `/office-hours`: Brainstorming and writing design docs
- `/plan-ceo-review`: CEO-level scope and product review
- `/plan-eng-review`: Engineering, schema, and security review
- `/plan-design-review`: Design review
- `/design-consultation`: Consulting on UI/UX
- `/review`: Code review and linting
- `/qa`: End-to-end browser and feature testing
- `/qa-only`: Pure QA testing without reviews
- `/ship`: Preparing for merge
- `/land-and-deploy`: Post-merge deployment checks
- `/canary`: Canary deployment verification
- `/benchmark`: Performance testing
- `/browse`: Web browsing and research
- `/retro`: Project retrospectives
- `/investigate`: Deep dive bug analysis
- `/setup-*`: Setup utilities for configuration

## Troubleshooting
If the `gstack` skills are not recognized or are failing to run, execute the `setup` script from the gstack installation folder (`~/.agents/skills/gstack/setup` or the local `.agents/skills` copy) to re-register them with the current environment.
