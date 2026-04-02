#!/bin/bash
ISSUE_NUMBER=$1
COMMENT=$2
gh issue comment "$ISSUE_NUMBER" --body "$COMMENT"
