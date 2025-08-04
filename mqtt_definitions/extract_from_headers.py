#!/usr/bin/env python3
"""
MQTT JSON Extractor

Extracts MQTT definitions from hand-written header files and generates
a master mqtt_definitions.json file.

This script reads the specific MQTT header files for each executable
and creates a consolidated JSON configuration.

Usage:
    python3 extract_from_headers.py [output_file]
    
    output_file: Output JSON file (default: mqtt_definitions_extracted.json)
"""

import json
import os
import re
import sys
from datetime import datetime
from pathlib import Path

class MQTTHeaderExtractor:
    def __init__(self):
        self.definitions = {
            "metadata": {
                "version": "1.0.0",
                "description": "MQTT definitions extracted from source header files",
                "extraction_date": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                "source": "Hand-written header files in src/"
            },
            "executables": {},
            "topics": {
                "published": {},
                "subscribed": {}
            },
            "message_types": {
                "json": {}
            }
        }
    
    def extract_from_header(self, header_file, executable_name):
        """Extract MQTT definitions from a single header file."""
        print(f"Extracting from {header_file}...")
        
        with open(header_file, 'r') as f:
            content = f.read()
        
        executable_def = {
            "name": executable_name,
            "header_file": str(header_file),
            "published_topics": [],
            "subscribed_topics": [],
            "message_types": [],
            "configuration": {}
        }
        
        # Extract configuration constants
        config_patterns = [
            (r'#define\s+(\w+)_MQTT_BASE_TOPIC\s+"([^"]+)"', 'base_topic'),
            (r'#define\s+(\w+)_MQTT_CLIENT_ID\s+"([^"]+)"', 'client_id'),
            (r'#define\s+(\w+)_HEARTBEAT_INTERVAL_SEC\s+(\d+)', 'heartbeat_interval')
        ]
        
        for pattern, key in config_patterns:
            matches = re.findall(pattern, content)
            if matches:
                executable_def["configuration"][key] = matches[0][1]
        
        # Extract published topics
        published_pattern = r'#define\s+\w+_TOPIC_(\w+)\s+"([^"]+)"\s*//\s*(.+)'
        published_matches = re.findall(published_pattern, content)
        
        for topic_name, topic_path, description in published_matches:
            topic_key = f"{executable_name}_{topic_name.lower()}"
            topic_def = {
                "path": topic_path,
                "description": description.strip(),
                "executable": executable_name,
                "type": "published"
            }
            
            self.definitions["topics"]["published"][topic_key] = topic_def
            executable_def["published_topics"].append(topic_key)
        
        # Extract subscribed topics (look for external topic references)
        subscribed_pattern = r'#define\s+\w+_TOPIC_\w+\s+"([^"]+)"\s*//\s*(.+)'
        external_pattern = r'"smartmower/(\w+)/(\w+)"'
        
        external_matches = re.findall(external_pattern, content)
        for system, topic in external_matches:
            topic_key = f"{system}_{topic}"
            if topic_key not in [t for topics in self.definitions["topics"]["subscribed"].values() for t in topics]:
                executable_def["subscribed_topics"].append(topic_key)
        
        # Extract JSON message types
        json_pattern = r'#define\s+\w+_JSON_(\w+)\s+"([^"]+)"'
        json_matches = re.findall(json_pattern, content)
        
        for msg_name, msg_string in json_matches:
            msg_key = f"{executable_name}_{msg_name.lower()}"
            self.definitions["message_types"]["json"][msg_key] = msg_string
            executable_def["message_types"].append(msg_key)
        
        # Extract JSON structure documentation from comments
        executable_def["json_structures"] = self._extract_json_structures(content)
        
        self.definitions["executables"][executable_name] = executable_def
        return executable_def
    
    def _extract_json_structures(self, content):
        """Extract JSON structure documentation from header comments."""
        structures = {}
        
        # Look for JSON structure documentation in comments
        structure_pattern = r'/\*\s*\n([^*]+MESSAGE[^*]+)\*/'
        matches = re.findall(structure_pattern, content, re.DOTALL)
        
        for match in matches:
            lines = match.strip().split('\n')
            current_structure = None
            current_json = []
            
            for line in lines:
                line = line.strip()
                if 'MESSAGE (' in line and '):' in line:
                    # Extract message name and topic
                    msg_match = re.search(r'(\w+)\s+MESSAGE\s+\(([^)]+)\):', line)
                    if msg_match:
                        current_structure = msg_match.group(1).lower()
                        structures[current_structure] = {
                            "topic": msg_match.group(2),
                            "json_example": []
                        }
                        current_json = structures[current_structure]["json_example"]
                elif line.startswith('{') or line.startswith('}') or '"' in line:
                    if current_json is not None:
                        current_json.append(line)
        
        return structures
    
    def extract_all_headers(self, src_directory):
        """Extract from all MQTT header files in the src directory."""
        src_path = Path(src_directory)
        
        # Define header files for each executable
        header_files = [
            (src_path / "pico" / "pico_mqtt.h", "pico"),
            (src_path / "gps" / "gps_mqtt.h", "gps"),
            (src_path / "fusion" / "fusion_mqtt.h", "fusion"),
            (src_path / "slam" / "slam_mqtt.h", "slam"),
            (src_path / "vision" / "vision_mqtt.h", "vision"),
            (src_path / "state_machine" / "state_machine_mqtt.h", "state_machine")
        ]
        
        for header_file, executable_name in header_files:
            if header_file.exists():
                self.extract_from_header(header_file, executable_name)
            else:
                print(f"Warning: Header file {header_file} not found")
    
    def generate_summary(self):
        """Generate a summary of extracted definitions."""
        summary = {
            "total_executables": len(self.definitions["executables"]),
            "total_published_topics": len(self.definitions["topics"]["published"]),
            "total_subscribed_topics": len(self.definitions["topics"]["subscribed"]),
            "total_message_types": len(self.definitions["message_types"]["json"]),
            "executables_summary": {}
        }
        
        for name, exec_def in self.definitions["executables"].items():
            summary["executables_summary"][name] = {
                "published_topics": len(exec_def["published_topics"]),
                "subscribed_topics": len(exec_def["subscribed_topics"]),
                "message_types": len(exec_def["message_types"])
            }
        
        return summary
    
    def save_json(self, output_file):
        """Save extracted definitions to JSON file."""
        with open(output_file, 'w') as f:
            json.dump(self.definitions, f, indent=2)
        
        print(f"Extracted definitions saved to: {output_file}")
        
        # Print summary
        summary = self.generate_summary()
        print(f"\nExtraction Summary:")
        print(f"- Total executables: {summary['total_executables']}")
        print(f"- Total published topics: {summary['total_published_topics']}")
        print(f"- Total message types: {summary['total_message_types']}")
        
        print(f"\nPer-executable breakdown:")
        for name, stats in summary["executables_summary"].items():
            print(f"  {name}:")
            print(f"    - Published topics: {stats['published_topics']}")
            print(f"    - Subscribed topics: {stats['subscribed_topics']}")
            print(f"    - Message types: {stats['message_types']}")

def main():
    """Main function to extract MQTT definitions from headers."""
    output_file = sys.argv[1] if len(sys.argv) > 1 else "mqtt_definitions_extracted.json"
    
    # Get the src directory (relative to this script)
    script_dir = Path(__file__).parent
    src_directory = script_dir.parent / "src"
    
    if not src_directory.exists():
        print(f"Error: Source directory '{src_directory}' not found!")
        sys.exit(1)
    
    print(f"Extracting MQTT definitions from headers in: {src_directory}")
    print(f"Output file: {output_file}")
    print()
    
    # Create extractor and process all headers
    extractor = MQTTHeaderExtractor()
    extractor.extract_all_headers(src_directory)
    
    # Save results
    extractor.save_json(output_file)
    
    print(f"\nExtraction completed successfully!")
    print(f"You can now use '{output_file}' as your master MQTT definitions file.")

if __name__ == "__main__":
    main()
