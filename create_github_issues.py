import json
import requests
import os

REPO_OWNER = "wholemealbaby"
REPO_NAME = "par-ass2"
API_BASE_URL = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/issues"

# Read issues from issues.json
try:
    with open("issues.json", "r", encoding="utf-8") as f:
        issues_data = json.load(f)
except FileNotFoundError:
    print("Error: issues.json not found.")
    exit(1)
except json.JSONDecodeError:
    print("Error: Could not decode issues.json. Please ensure it's valid JSON.")
    exit(1)

# Prompt for GitHub Personal Access Token
github_token = os.environ.get("GITHUB_TOKEN")
if not github_token:
    github_token = input("Please enter your GitHub Personal Access Token (PAT) with 'repo' scope: ")
    if not github_token:
        print("Error: GitHub PAT not provided. Exiting.")
        exit(1)

headers = {
    "Authorization": f"token {github_token}",
    "Accept": "application/vnd.github.v3+json"
}

print(f"Attempting to create {len(issues_data)} issues in {REPO_OWNER}/{REPO_NAME}...")

for i, issue in enumerate(issues_data):
    title = issue.get("title")
    body = issue.get("body")
    labels = issue.get("labels", [])

    if not title:
        print(f"Skipping issue {i+1} due to missing title.")
        continue

    data = {
        "title": title,
        "body": body,
        "labels": labels
    }

    try:
        response = requests.post(API_BASE_URL, headers=headers, json=data)
        response.raise_for_status()  # Raise an exception for HTTP errors
        created_issue = response.json()
        print(f"Successfully created issue #{created_issue['number']}: {created_issue['title']}")
    except requests.exceptions.RequestException as e:
        print(f"Error creating issue '{title}': {e}")
        if response is not None:
            print(f"Response status code: {response.status_code}")
            print(f"Response content: {response.text}")
    except Exception as e:
        print(f"An unexpected error occurred for issue '{title}': {e}")

print("Issue creation process completed.")
