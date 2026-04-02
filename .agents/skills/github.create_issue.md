---
name: github_create_issue
description: Create a new GitHub issue
inputs:
  - name: title
    type: string
  - name: body
    type: string
---

Command:
gh_create_issue.sh "{{title}}" "{{body}}"
