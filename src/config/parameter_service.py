#!/usr/bin/env python3
"""
Parameter Service for Smart Mower Robot
Service that runs at boot to manage robot parameters via MQTT
"""

import os
import sys
import signal
import logging
import time
from pathlib import Path
from parameter_manager import ParameterManager

class ParameterService:
    """Service wrapper for parameter manager"""
    
    def __init__(self, config_path: str = None):
        # Setup paths
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "robot_config.json")
        
        self.config_path = config_path
        self.parameter_manager = None
        self.running = False
        
        # Setup logging
        log_dir = "/opt/smartmower/log"
        os.makedirs(log_dir, exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(f"{log_dir}/parameter_service.log"),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Setup signal handlers
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.info(f"Received signal {signum}, shutting down...")
        self.stop()
    
    def start(self):
        """Start the parameter service"""
        try:
            self.logger.info("Starting Parameter Service...")
            
            # Create parameter manager
            self.parameter_manager = ParameterManager(self.config_path)
            
            # Register parameter change callbacks
            self._register_callbacks()
            
            self.running = True
            self.logger.info("Parameter Service started successfully")
            
            # Main service loop
            while self.running:
                time.sleep(1)
                
        except Exception as e:
            self.logger.error(f"Error starting parameter service: {e}")
            return False
        
        return True
    
    def stop(self):
        """Stop the parameter service"""
        self.running = False
        if self.parameter_manager:
            self.parameter_manager.shutdown()
        self.logger.info("Parameter Service stopped")
    
    def _register_callbacks(self):
        """Register callbacks for important parameter changes"""
        
        def on_pid_change(new_value, old_value):
            self.logger.info(f"PID parameter changed: {old_value} -> {new_value}")
            # Here you would notify the control system about PID changes
        
        def on_speed_change(new_value, old_value):
            self.logger.info(f"Speed parameter changed: {old_value} -> {new_value}")
            # Here you would notify the motion controller about speed changes
        
        def on_battery_change(new_value, old_value):
            self.logger.info(f"Battery parameter changed: {old_value} -> {new_value}")
            # Here you would notify the battery monitor about changes
        
        # Register callbacks
        pid_params = [
            "tuning/pid/linear_kp", "tuning/pid/linear_ki", "tuning/pid/linear_kd",
            "tuning/pid/angular_kp", "tuning/pid/angular_ki", "tuning/pid/angular_kd"
        ]
        
        for param in pid_params:
            self.parameter_manager.register_callback(param, on_pid_change)
        
        speed_params = [
            "tuning/speeds/max_linear_speed", "tuning/speeds/max_angular_speed",
            "tuning/speeds/cutting_speed"
        ]
        
        for param in speed_params:
            self.parameter_manager.register_callback(param, on_speed_change)
        
        battery_params = [
            "battery/type", "battery/cell_count", "battery/capacity"
        ]
        
        for param in battery_params:
            self.parameter_manager.register_callback(param, on_battery_change)

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Smart Mower Parameter Service")
    parser.add_argument("--config", help="Path to configuration file")
    parser.add_argument("--daemon", action="store_true", help="Run as daemon")
    
    args = parser.parse_args()
    
    # Create and start service
    service = ParameterService(args.config)
    
    if args.daemon:
        # Daemonize the process
        try:
            pid = os.fork()
            if pid > 0:
                # Parent process exits
                sys.exit(0)
        except OSError as e:
            print(f"Fork failed: {e}")
            sys.exit(1)
        
        # Decouple from parent environment
        os.chdir("/")
        os.setsid()
        os.umask(0)
        
        # Second fork
        try:
            pid = os.fork()
            if pid > 0:
                sys.exit(0)
        except OSError as e:
            print(f"Second fork failed: {e}")
            sys.exit(1)
        
        # Redirect standard file descriptors
        sys.stdout.flush()
        sys.stderr.flush()
        
        with open('/dev/null', 'r') as si:
            os.dup2(si.fileno(), sys.stdin.fileno())
        with open('/dev/null', 'a+') as so:
            os.dup2(so.fileno(), sys.stdout.fileno())
        with open('/dev/null', 'a+') as se:
            os.dup2(se.fileno(), sys.stderr.fileno())
    
    # Start the service
    service.start()

if __name__ == "__main__":
    main()
