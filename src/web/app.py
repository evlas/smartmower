#!/usr/bin/env python3
"""
Smart Mower Web Interface
Professional web application for managing robot configuration and services
"""

import os
import json
import subprocess
import logging
import shutil
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Any, Optional
from dataclasses import dataclass

from flask import Flask, render_template, request, jsonify, redirect, url_for, flash, send_file
from werkzeug.utils import secure_filename

# Configuration
CONFIG_FILE = '/opt/smartmower/etc/robot_config.json'
BACKUP_DIR = '/opt/smartmower/data/backups'
LOG_DIR = '/opt/smartmower/log'
WEB_LOG_FILE = '/opt/smartmower/log/web_interface.log'

# Ensure directories exist
os.makedirs(BACKUP_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(WEB_LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.secret_key = 'smartmower-web-interface-2024'

@dataclass
class BackupInfo:
    """Information about a configuration backup"""
    filename: str
    name: str
    description: str
    created_at: datetime
    size: int
    auto_created: bool = False

# Services to manage
SERVICES = [
    "smartmower-pico",
    "smartmower-gps", 
    "smartmower-fusion",
    "smartmower-path-planning",
    "smartmower-slam",
    "smartmower-state-machine",
    "smartmower-vision-camera",
    "smartmower-vision-grass",
    "smartmower-vision-obstacle",
    "smartmower-vision-perimeter"
]

    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class ConfigManager:
    """Manages robot configuration operations"""
    
    def __init__(self, config_file):
        self.config_file = config_file
        self.ensure_backup_dir()
    
    def ensure_backup_dir(self):
        """Ensure backup directory exists"""
        os.makedirs(BACKUP_DIR, exist_ok=True)
    
    def load_config(self):
        """Load current configuration"""
        try:
            with open(self.config_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Error loading config: {e}")
            return None
    
    def save_config(self, config_data):
        """Save configuration with backup"""
        try:
            # Create backup
            self.create_backup()
            
            # Save new configuration
            with open(self.config_file, 'w') as f:
                json.dump(config_data, f, indent=2)
            
            logger.info("Configuration saved successfully")
            return True
        except Exception as e:
            logger.error(f"Error saving config: {e}")
            return False
    
    def create_backup(self):
        """Create backup of current configuration"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_file = os.path.join(BACKUP_DIR, f"robot_config_{timestamp}.json")
            
            if os.path.exists(self.config_file):
                subprocess.run(['cp', self.config_file, backup_file], check=True)
                logger.info(f"Backup created: {backup_file}")
        except Exception as e:
            logger.error(f"Error creating backup: {e}")
    
    def get_backups(self):
        """Get list of available backups"""
        try:
            backups = []
            for file in os.listdir(BACKUP_DIR):
                if file.startswith("robot_config_") and file.endswith(".json"):
                    file_path = os.path.join(BACKUP_DIR, file)
                    stat = os.stat(file_path)
                    backups.append({
                        'filename': file,
                        'timestamp': datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
                        'size': stat.st_size
                    })
            return sorted(backups, key=lambda x: x['timestamp'], reverse=True)
        except Exception as e:
            logger.error(f"Error getting backups: {e}")
            return []
    
    def restore_backup(self, backup_filename):
        """Restore configuration from backup"""
        try:
            backup_path = os.path.join(BACKUP_DIR, backup_filename)
            if os.path.exists(backup_path):
                subprocess.run(['cp', backup_path, self.config_file], check=True)
                logger.info(f"Configuration restored from {backup_filename}")
                return True
            return False
        except Exception as e:
            logger.error(f"Error restoring backup: {e}")
            return False

class ServiceManager:
    """Manages systemd services"""
    
    @staticmethod
    def get_service_status(service_name):
        """Get status of a systemd service"""
        try:
            result = subprocess.run(
                ['systemctl', 'is-active', service_name],
                capture_output=True, text=True
            )
            return result.stdout.strip()
        except Exception as e:
            logger.error(f"Error getting service status for {service_name}: {e}")
            return "unknown"
    
    @staticmethod
    def get_all_services_status():
        """Get status of all Smart Mower services"""
        status = {}
        for service in SERVICES:
            status[service] = ServiceManager.get_service_status(service)
        return status
    
    @staticmethod
    def restart_service(service_name):
        """Restart a systemd service"""
        try:
            result = subprocess.run(
                ['sudo', 'systemctl', 'restart', service_name],
                capture_output=True, text=True
            )
            if result.returncode == 0:
                logger.info(f"Service {service_name} restarted successfully")
                return True
            else:
                logger.error(f"Error restarting {service_name}: {result.stderr}")
                return False
        except Exception as e:
            logger.error(f"Error restarting service {service_name}: {e}")
            return False
    
    @staticmethod
    def restart_all_services():
        """Restart all Smart Mower services"""
        results = {}
        for service in SERVICES:
            results[service] = ServiceManager.restart_service(service)
        return results

# Initialize managers
config_manager = ConfigManager(CONFIG_FILE)
service_manager = ServiceManager()

@app.route('/')
def index():
    """Main dashboard"""
    services_status = service_manager.get_all_services_status()
    return render_template('index.html', services_status=services_status)

@app.route('/config')
def config_view():
    """Configuration management page"""
    config_data = config_manager.load_config()
    if config_data is None:
        flash('Error loading configuration file', 'error')
        return redirect(url_for('index'))
    
    return render_template('config.html', config_data=config_data)

@app.route('/config/save', methods=['POST'])
def config_save():
    """Save configuration changes"""
    try:
        config_data = request.get_json()
        if config_manager.save_config(config_data):
            flash('Configuration saved successfully', 'success')
            return jsonify({'success': True})
        else:
            flash('Error saving configuration', 'error')
            return jsonify({'success': False, 'error': 'Save failed'})
    except Exception as e:
        logger.error(f"Error in config_save: {e}")
        flash('Error processing configuration', 'error')
        return jsonify({'success': False, 'error': str(e)})

@app.route('/services')
def services_view():
    """Services management page"""
    services_status = service_manager.get_all_services_status()
    return render_template('services.html', services_status=services_status)

@app.route('/services/restart/<service_name>', methods=['POST'])
def restart_service(service_name):
    """Restart a specific service"""
    if service_name in SERVICES:
        success = service_manager.restart_service(service_name)
        if success:
            flash(f'Service {service_name} restarted successfully', 'success')
        else:
            flash(f'Error restarting service {service_name}', 'error')
    else:
        flash('Invalid service name', 'error')
    
    return redirect(url_for('services_view'))

@app.route('/services/restart-all', methods=['POST'])
def restart_all_services():
    """Restart all services"""
    results = service_manager.restart_all_services()
    success_count = sum(1 for success in results.values() if success)
    total_count = len(results)
    
    if success_count == total_count:
        flash(f'All {total_count} services restarted successfully', 'success')
    else:
        flash(f'{success_count}/{total_count} services restarted successfully', 'warning')
    
    return redirect(url_for('services_view'))

@app.route('/backups')
def backups_view():
    """Backups management page"""
    backups = config_manager.get_backups()
    return render_template('backups.html', backups=backups)

@app.route('/backups/restore/<backup_filename>', methods=['POST'])
def restore_backup(backup_filename):
    """Restore configuration from backup"""
    if config_manager.restore_backup(backup_filename):
        flash(f'Configuration restored from {backup_filename}', 'success')
    else:
        flash('Error restoring backup', 'error')
    
    return redirect(url_for('backups_view'))

@app.route('/api/services/status')
def api_services_status():
    """API endpoint for services status"""
    return jsonify(service_manager.get_all_services_status())

@app.route('/api/config')
def api_config():
    """API endpoint for configuration"""
    config_data = config_manager.load_config()
    if config_data:
        return jsonify(config_data)
    else:
        return jsonify({'error': 'Failed to load configuration'}), 500

if __name__ == '__main__':
    # Ensure log directory exists
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    
    # Run Flask app
    app.run(host='0.0.0.0', port=8080, debug=False)
