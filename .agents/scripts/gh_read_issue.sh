#!/bin/bash
ISSUE_NUMBER=$1
gh issue view "$ISSUE_NUMBER" --json title,body,state,labels,comments
