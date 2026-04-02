#!/bin/bash
ISSUE_NUMBER=$1
TITLE=$2
BODY=$3
gh issue edit "$ISSUE_NUMBER" --title "$TITLE" --body "$BODY"
