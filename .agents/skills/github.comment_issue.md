---
name: github_comment_issue
description: Add a comment to an issue
inputs:
  - name: issue_number
    type: string
  - name: comment
    type: string
---

Command:
gh_comment_issue.sh {{issue_number}} "{{comment}}"
