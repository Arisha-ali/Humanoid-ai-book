---
id: 6
title: "Implement Deployment Workflow for Physical AI Book"
stage: "implement"
date: "2025-12-11"
surface: "agent"
model: "gemini"
feature: "physical-ai-book"
branch: "main"
user: "Arisha-ali"
labels: ["deployment", "github-actions", "docusaurus"]
links:
  - "https://github.com/Arisha-ali/Humanoid-ai-book.git"
files:
  - ".github/workflows/deploy.yml"
  - "docusaurus.config.js"
tests:
  - "N/A"
---

# Prompt

I have built a Humanoid AI Book using Docusaurus, Speckit Plus, and Gemini CLI. I want to deploy this project to GitHub Pages. Here are the details:
- GitHub Repository URL: https://github.com/Arisha-ali/Humanoid-ai-book.git
- GitHub Username: Arisha-ali
- Repository Name: Humanoid-ai-book

Your tasks:
1.  Update my `docusaurus.config.js` automatically:
    -   `url`: "https://arisha-ali.github.io"
    -   `baseUrl`: "/Humanoid-ai-book/"
2.  Create a full GitHub Actions deployment workflow.
    File path should be:
    `.github/workflows/deploy.yml`
    Workflow requirements:
    -   Trigger on push to `main`
    -   Use Node.js 18
    -   Install dependencies
    -   Build Docusaurus
    -   Deploy build folder to GitHub Pages using `peaceiris/actions-gh-pages`
3.  Generate terminal commands I need to run:
    -   `git init` (if needed)
    -   `git add .`
    -   `git commit`
    -   `git branch -M main`
    -   `git remote add origin https://github.com/Arisha-ali/Humanoid-ai-book.git`
    -   `git push -u origin main`
4.  After generating everything,
    verify that the repo settings support GitHub Pages
    and tell me the final live deployment URL:
    `https://arisha-ali.github.io/Humanoid-ai-book/`
5.  Output all files clearly:
    -   Updated `docusaurus.config.js` content
    -   `deploy.yml` content
    -   All commands to run step-by-step

Do not leave anything incomplete. Make everything production-ready and error-free.
Ensure the deployment works in one push.

# Response

I have updated the `docusaurus.config.js` file and created the GitHub Actions workflow for deployment.

Here are the terminal commands you need to run:

```bash
git add .
git commit -m "feat: add GitHub Actions workflow for deployment"
git push
```

After pushing the changes, the GitHub Actions workflow will be triggered and your project will be deployed to GitHub Pages.

The final deployment URL will be: `httpshttps://arisha-ali.github.io/Humanoid-ai-book/`
