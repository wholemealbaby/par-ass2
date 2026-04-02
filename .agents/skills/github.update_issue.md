---
name: github_update_issue
description: Update an existing GitHub issue
inputs:
  - name: issue_number
    type: string
  - name: title
    type: string
  - name: body
    type: string
---

Command:
gh_update_issue.sh {{issue_number}} "{{title}}" "{{body}}"
