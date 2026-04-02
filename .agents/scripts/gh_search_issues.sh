#!/bin/bash
QUERY=$1
gh issue list --search "$QUERY" --json number,title,state
