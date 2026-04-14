#!/bin/bash

# Script to read GitHub issues with full descriptions and save to JSON file
# Uses gh CLI to fetch full issue details
# Usage: read_issues_details.sh [-i input_file] [-o output_file] [-h]

set -e  # Exit on error

# Default file paths
INPUT_FILE="issues.json"
OUTPUT_FILE="issues_details.json"
SHOW_HELP=false

# Function to display usage
usage() {
    cat << EOF
Usage: $0 [-i input_file] [-o output_file] [-h]

Fetch full details for GitHub issues listed in a JSON input file and save to output file.

Options:
  -i FILE    Input JSON file containing issues array (default: issues.json)
  -o FILE    Output JSON file for detailed issues (default: issues_details.json)
  -h         Show this help message

Example:
  $0 -i my_issues.json -o detailed_issues.json
  $0 -i navigation_issues.json -o navigation_details.json
  $0 -o all_issues_details.json

Input JSON format:
  [{"number": 10, "state": "OPEN", "title": "Issue title"}, ...]
EOF
}

# Parse command-line arguments
while getopts "i:o:h" opt; do
    case $opt in
        i)
            INPUT_FILE="$OPTARG"
            ;;
        o)
            OUTPUT_FILE="$OPTARG"
            ;;
        h)
            SHOW_HELP=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            exit 1
            ;;
    esac
done

# Show help if requested
if [ "$SHOW_HELP" = true ]; then
    usage
    exit 0
fi

echo "Reading issues from $INPUT_FILE..."

# Check if input file exists
if [ ! -f "$INPUT_FILE" ]; then
    echo "Error: Input file $INPUT_FILE not found!"
    exit 1
fi

# Check if gh CLI is available
if ! command -v gh &> /dev/null; then
    echo "Error: GitHub CLI (gh) is not installed or not in PATH"
    exit 1
fi

# Check if user is authenticated with GitHub CLI
if ! gh auth status &> /dev/null; then
    echo "Error: Not authenticated with GitHub CLI. Please run 'gh auth login' first."
    exit 1
fi

# Read the input JSON file
if [ ! -s "$INPUT_FILE" ]; then
    echo "Error: Input file $INPUT_FILE is empty!"
    exit 1
fi

# Parse the JSON array from input file
ISSUE_DATA=$(cat "$INPUT_FILE")

# Check if it's valid JSON
if ! echo "$ISSUE_DATA" | jq . > /dev/null 2>&1; then
    echo "Error: Input file $INPUT_FILE does not contain valid JSON"
    exit 1
fi

# Get the length of the array
ISSUE_COUNT=$(echo "$ISSUE_DATA" | jq '. | length')

echo "Found $ISSUE_COUNT issues to process..."

# Create an empty array for the detailed issues
DETAILED_ISSUES="[]"

# Process each issue
for ((i=0; i<ISSUE_COUNT; i++)); do
    # Extract issue number
    ISSUE_NUMBER=$(echo "$ISSUE_DATA" | jq -r ".[$i].number")
    ISSUE_TITLE=$(echo "$ISSUE_DATA" | jq -r ".[$i].title")
    ISSUE_STATE=$(echo "$ISSUE_DATA" | jq -r ".[$i].state")
    
    echo "Processing issue #$ISSUE_NUMBER: $ISSUE_TITLE"
    
    # Fetch full issue details using GitHub CLI
    # Include all relevant fields for comprehensive details (excluding invalid fields)
    ISSUE_DETAILS=$(gh issue view "$ISSUE_NUMBER" --json \
        number,title,body,state,labels,comments,createdAt,updatedAt,url,assignees,milestone \
        2>&1)
    
    # Check if the command succeeded (exit code 0) and produced valid JSON
    if [ $? -eq 0 ] && echo "$ISSUE_DETAILS" | jq . > /dev/null 2>&1; then
        # Success - we have valid JSON
        echo "  Successfully fetched full details"
    else
        echo "Warning: Could not fetch details for issue #$ISSUE_NUMBER. Using basic info."
        # Create a basic object with the info we have
        BASIC_INFO=$(jq -n \
            --arg number "$ISSUE_NUMBER" \
            --arg title "$ISSUE_TITLE" \
            --arg state "$ISSUE_STATE" \
            '{number: ($number | tonumber), title: $title, state: $state, error: "Failed to fetch full details"}')
        ISSUE_DETAILS="$BASIC_INFO"
    fi
    
    # Add the issue details to our array
    DETAILED_ISSUES=$(echo "$DETAILED_ISSUES" | jq --argjson issue "$ISSUE_DETAILS" '. + [$issue]')
    
    # Add a small delay to avoid rate limiting
    if [ $i -lt $((ISSUE_COUNT-1)) ]; then
        sleep 0.5
    fi
done

# Write the detailed issues to output file
echo "$DETAILED_ISSUES" | jq '.' > "$OUTPUT_FILE"

echo "Successfully processed $ISSUE_COUNT issues."
echo "Detailed issues saved to $OUTPUT_FILE"

# Display summary
echo ""
echo "Summary of issues saved:"
echo "$DETAILED_ISSUES" | jq -r '.[] | "#\(.number): \(.title) (\(.state))"'

# Check file size
FILE_SIZE=$(wc -c < "$OUTPUT_FILE")
echo ""
echo "Output file size: $FILE_SIZE bytes"