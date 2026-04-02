import json
import requests

# Load issues
with open('issues.json', 'r') as f:
    issues = json.load(f)

# GitHub configuration
TOKEN = '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'
REPO = 'wholemealbaby/par-ass2'
HEADERS = {
    'Authorization': f'token {TOKEN}',
    'Accept': 'application/vnd.github.v3+json'
}

def create_issue(issue):
    url = f'https://api.github.com/repos/{REPO}/issues'
    data = {
        'title': issue['title'],
        'body': issue['body'],
        'labels': issue.get('labels', [])
    }
    response = requests.post(url, headers=HEADERS, json=data)
    if response.status_code == 201:
        print(f"Successfully created issue: {issue['title']}")
    else:
        print(f"Failed to create issue: {issue['title']}. Status: {response.status_code}, Response: {response.text}")

for issue in issues:
    create_issue(issue)
