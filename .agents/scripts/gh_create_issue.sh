#!/bin/bash
TITLE=$1
BODY=$2
gh issue create --title "$TITLE" --body "$BODY"
