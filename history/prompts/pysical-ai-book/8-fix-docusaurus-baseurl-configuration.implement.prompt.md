---
id: 8
title: "Fix Docusaurus baseUrl Configuration"
stage: implement
date: 2025-12-14T22:45:00.000Z
surface: "agent"
model: "gemini-cli"
feature: "physical-ai-book"
branch: "main"
user: "unknown"
command: "solve baseUrl error"
labels: ["bug-fix", "docusaurus", "configuration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.js
tests:
  - "npm run build"
---

# Prompt

Your Docusaurus site did not load properly.A very common reason is a wrong site baseUrl configuration.Current configured baseUrl = /Humanoid-ai-book/We suggest trying baseUrl = /

# Response

I have modified the `docusaurus.config.js` file to change the `baseUrl` from `/Humanoid-ai-book/` to `/`. The project now builds successfully. Please try deploying to Vercel again.
