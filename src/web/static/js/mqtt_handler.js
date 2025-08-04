/**
 * MQTT Handler for Smart Mower Web Interface
 * Provides real-time updates and command sending via MQTT
 */

class MqttHandler {
    constructor() {
        this.client = null;
        this.connected = false;
        this.topics = {
            STATE: 'smartmower/state',          // Current state (IDLE, MOWING, DOCKING, etc.)
            BATTERY: 'smartmower/battery',      // Battery percentage (0-100)
            INFO: 'smartmower/info',            // Additional info (JSON)
            COMMAND: 'smartmower/command',      // Commands to send
            SENSORS: 'smartmower/sensors',      // Sensor data
            STATUS: 'smartmower/status'         // System status
        };
        
        // Callbacks for different message types
        this.callbacks = {
            state: [],
            battery: [],
            info: [],
            sensors: [],
            status: []
        };
        
        this.init();
    }
    
    /**
     * Initialize MQTT connection
     */
    init() {
        // Get MQTT config from server
        fetch('/api/mqtt-config')
            .then(response => response.json())
            .then(config => {
                this.connect(config);
            })
            .catch(error => {
                console.error('Failed to load MQTT config:', error);
                // Fallback to default config
                this.connect({
                    host: window.location.hostname,
                    port: 9001,  // Default MQTT over WebSocket port
                    username: 'web',
                    password: 'smart'
                });
            });
    }
    
    /**
     * Connect to MQTT broker
     * @param {Object} config - MQTT connection config
     */
    connect(config) {
        // Create client
        this.client = new Paho.Client(
            config.host,
            config.port,
            `web_${Math.random().toString(16).substr(2, 8)}`
        );
        
        // Set callback handlers
        this.client.onConnectionLost = this.onConnectionLost.bind(this);
        this.client.onMessageArrived = this.onMessageArrived.bind(this);
        
        // Connect to the broker
        const connectOptions = {
            onSuccess: this.onConnect.bind(this),
            onFailure: this.onConnectFailure.bind(this),
            userName: config.username,
            password: config.password,
            useSSL: window.location.protocol === 'https:',
            reconnect: true,
            keepAliveInterval: 30,
            cleanSession: true
        };
        
        this.client.connect(connectOptions);
    }
    
    /**
     * Handle successful connection
     */
    onConnect() {
        console.log('MQTT Connected');
        this.connected = true;
        
        // Subscribe to topics
        Object.values(this.topics).forEach(topic => {
            this.client.subscribe(topic, { qos: 0 });
        });
        
        // Request initial state
        this.publish(this.topics.COMMAND, 'GET_STATE');
    }
    
    /**
     * Handle connection failure
     * @param {Object} error - Connection error
     */
    onConnectFailure(error) {
        console.error('MQTT Connection failed:', error);
        this.connected = false;
        
        // Try to reconnect after delay
        setTimeout(() => this.init(), 5000);
    }
    
    /**
     * Handle lost connection
     * @param {Object} response - Response object
     */
    onConnectionLost(response) {
        console.log('MQTT Connection lost:', response);
        this.connected = false;
        
        // Try to reconnect
        this.init();
    }
    
    /**
     * Handle incoming messages
     * @param {Object} message - MQTT message
     */
    onMessageArrived(message) {
        try {
            const topic = message.destinationName;
            const payload = message.payloadString;
            
            // Parse JSON payloads
            let data;
            try {
                data = JSON.parse(payload);
            } catch (e) {
                data = payload;
            }
            
            // Call appropriate callbacks
            if (topic === this.topics.STATE) {
                this.callbacks.state.forEach(cb => cb(data));
            } else if (topic === this.topics.BATTERY) {
                this.callbacks.battery.forEach(cb => cb(parseFloat(data)));
            } else if (topic === this.topics.INFO) {
                this.callbacks.info.forEach(cb => cb(data));
            } else if (topic === this.topics.SENSORS) {
                this.callbacks.sensors.forEach(cb => cb(data));
            } else if (topic === this.topics.STATUS) {
                this.callbacks.status.forEach(cb => cb(data));
            }
            
        } catch (error) {
            console.error('Error processing MQTT message:', error);
        }
    }
    
    /**
     * Publish a message to a topic
     * @param {string} topic - MQTT topic
     * @param {string|Object} message - Message to send
     */
    publish(topic, message) {
        if (!this.connected) {
            console.warn('MQTT not connected, cannot publish');
            return;
        }
        
        const payload = typeof message === 'object' 
            ? JSON.stringify(message) 
            : String(message);
            
        const mqttMessage = new Paho.Message(payload);
        mqttMessage.destinationName = topic;
        mqttMessage.qos = 0;
        
        this.client.send(mqttMessage);
    }
    
    /**
     * Send a command to the robot
     * @param {string} command - Command to send
     */
    sendCommand(command) {
        this.publish(this.topics.COMMAND, command);
    }
    
    /**
     * Register a callback for state updates
     * @param {Function} callback - Callback function
     */
    onStateUpdate(callback) {
        this.callbacks.state.push(callback);
    }
    
    /**
     * Register a callback for battery updates
     * @param {Function} callback - Callback function
     */
    onBatteryUpdate(callback) {
        this.callbacks.battery.push(callback);
    }
    
    /**
     * Register a callback for info updates
     * @param {Function} callback - Callback function
     */
    onInfoUpdate(callback) {
        this.callbacks.info.push(callback);
    }
}

// Initialize MQTT handler when the page loads
window.addEventListener('DOMContentLoaded', () => {
    window.mqttHandler = new MqttHandler();
});
