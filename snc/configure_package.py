#!/usr/bin/env python3
"""
Script to regenerate package.xml, setup.cfg, and resource file based on environment variable SNC_PACKAGE_NAME.
Run this script before building the package to adapt to a different package name.

Usage:
    ./configure_package.py [--package-name NAME] [--output-dir DIR]

If --package-name is not provided, reads from SNC_PACKAGE_NAME environment variable (default: 'snc').
If --output-dir is provided, operates on files in that directory (default: current directory).
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Configure package name in package.xml, setup.cfg, and resource file.')
    parser.add_argument('--package-name', help='Package name to use (overrides environment variable)')
    parser.add_argument('--output-dir', default='.', help='Directory containing package files (default: current directory)')
    return parser.parse_args()

def get_package_name(args):
    """Determine package name from args or environment."""
    if args.package_name:
        package_name = args.package_name
    else:
        package_name = os.environ.get('SNC_PACKAGE_NAME', 'snc')
    if not package_name.isidentifier():
        print(f"Warning: Package name '{package_name}' may not be a valid Python identifier.", file=sys.stderr)
    return package_name

def update_package_xml(package_name, base_dir):
    """Update or create package.xml with the given package name."""
    xml_path = base_dir / 'package.xml'
    if not xml_path.exists():
        print(f"Error: package.xml not found at {xml_path}.", file=sys.stderr)
        return False
    
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        # Find the <name> element
        name_elem = root.find('name')
        if name_elem is None:
            print("Error: <name> element not found in package.xml", file=sys.stderr)
            return False
        old_name = name_elem.text
        name_elem.text = package_name
        # Write back
        tree.write(xml_path, encoding='UTF-8', xml_declaration=True)
        print(f"Updated package.xml: changed package name from '{old_name}' to '{package_name}'")
        return True
    except Exception as e:
        print(f"Error updating package.xml: {e}", file=sys.stderr)
        return False

def update_setup_cfg(package_name, base_dir):
    """Update setup.cfg to use the new package name in script_dir and install_scripts."""
    cfg_path = base_dir / 'setup.cfg'
    if not cfg_path.exists():
        print(f"Warning: setup.cfg not found at {cfg_path}.", file=sys.stderr)
        return True
    
    try:
        with open(cfg_path, 'r') as f:
            content = f.read()
        # Detect current package name by looking for pattern script_dir=$base/lib/<name>
        import re
        # Simple detection: find the word after 'lib/' in script_dir line
        match = re.search(r'script_dir=\$base/lib/(\w+)', content)
        old_name = 'snc'  # default fallback
        if match:
            old_name = match.group(1)
        if old_name != package_name:
            # Replace all occurrences of old_name with package_name (whole word)
            # Use regex to avoid replacing substrings
            pattern = r'\b' + re.escape(old_name) + r'\b'
            new_content = re.sub(pattern, package_name, content)
            if new_content != content:
                with open(cfg_path, 'w') as f:
                    f.write(new_content)
                print(f"Updated setup.cfg: replaced '{old_name}' with '{package_name}'")
            else:
                print("No changes needed in setup.cfg")
        return True
    except Exception as e:
        print(f"Error updating setup.cfg: {e}", file=sys.stderr)
        return False

def update_resource_file(package_name, base_dir):
    """Rename resource file to match package name."""
    resource_dir = base_dir / 'resource'
    if not resource_dir.exists():
        print(f"Warning: resource directory not found at {resource_dir}.", file=sys.stderr)
        return True
    
    # Determine current resource file name by listing files in resource directory
    # Usually there is a single file with the package name.
    existing_files = list(resource_dir.iterdir())
    old_file = None
    if len(existing_files) == 1:
        old_file = existing_files[0]
    else:
        # Fallback: assume file named 'snc'
        old_file = resource_dir / 'snc'
    
    new_file = resource_dir / package_name
    if old_file.exists():
        if old_file == new_file:
            print(f"Resource file already named '{package_name}'")
            return True
        try:
            old_file.rename(new_file)
            print(f"Renamed resource file from '{old_file.name}' to '{new_file.name}'")
            return True
        except Exception as e:
            print(f"Error renaming resource file: {e}", file=sys.stderr)
            return False
    else:
        # Create empty resource file if it doesn't exist
        try:
            new_file.touch()
            print(f"Created empty resource file '{new_file.name}'")
            return True
        except Exception as e:
            print(f"Error creating resource file: {e}", file=sys.stderr)
            return False

def main():
    args = parse_args()
    base_dir = Path(args.output_dir).resolve()
    if not base_dir.exists():
        print(f"Error: output directory '{base_dir}' does not exist.", file=sys.stderr)
        sys.exit(1)
    
    package_name = get_package_name(args)
    print(f"Using package name: {package_name}")
    print(f"Operating in directory: {base_dir}")
    
    success = True
    success = update_package_xml(package_name, base_dir) and success
    success = update_setup_cfg(package_name, base_dir) and success
    success = update_resource_file(package_name, base_dir) and success
    
    if success:
        print("Configuration updated successfully.")
        sys.exit(0)
    else:
        print("Configuration partially failed.", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()