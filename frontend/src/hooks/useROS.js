import { useState, useEffect, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';

/**
 * Custom hook for managing ROS connection with automatic reconnection
 *
 * @param {Object} config - Configuration object
 * @param {string} config.url - ROS bridge server URL (default: 'ws://localhost:9090')
 * @param {number} config.reconnectInterval - Reconnection interval in ms (default: 3000)
 * @param {number} config.maxReconnectAttempts - Maximum reconnection attempts (default: 10)
 * @returns {Object} ROS connection state and utilities
 */
export const useROS = (config = {}) => {
  const {
    url = 'ws://localhost:9090',
    reconnectInterval = 3000,
    maxReconnectAttempts = 10
  } = config;

  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [reconnectAttempts, setReconnectAttempts] = useState(0);
  const [lastError, setLastError] = useState(null);

  const rosRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);

  // Initialize ROS connection
  const connect = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
    }

    setConnectionStatus('connecting');
    setLastError(null);

    rosRef.current = new ROSLIB.Ros({
      url: url
    });

    rosRef.current.on('connection', () => {
      console.log('Connected to ROS bridge server at', url);
      setIsConnected(true);
      setConnectionStatus('connected');
      setReconnectAttempts(0);
      reconnectAttemptsRef.current = 0;
    });

    rosRef.current.on('error', (error) => {
      console.error('ROS connection error:', error);
      setLastError(error);
      setConnectionStatus('error');
      setIsConnected(false);

      // Attempt reconnection if under max attempts
      if (reconnectAttemptsRef.current < maxReconnectAttempts) {
        reconnectAttemptsRef.current += 1;
        setReconnectAttempts(reconnectAttemptsRef.current);

        reconnectTimeoutRef.current = setTimeout(() => {
          console.log(`Attempting reconnection (${reconnectAttemptsRef.current}/${maxReconnectAttempts})`);
          connect();
        }, reconnectInterval);
      } else {
        setConnectionStatus('failed');
        console.error('Max reconnection attempts reached');
      }
    });

    rosRef.current.on('close', () => {
      console.log('ROS connection closed');
      setIsConnected(false);
      setConnectionStatus('disconnected');

      // Auto-reconnect if not manually disconnected
      if (reconnectAttemptsRef.current < maxReconnectAttempts) {
        reconnectTimeoutRef.current = setTimeout(() => {
          reconnectAttemptsRef.current += 1;
          setReconnectAttempts(reconnectAttemptsRef.current);
          console.log(`Attempting reconnection (${reconnectAttemptsRef.current}/${maxReconnectAttempts})`);
          connect();
        }, reconnectInterval);
      }
    });

  }, [url, reconnectInterval, maxReconnectAttempts]);

  // Disconnect from ROS
  const disconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }

    setIsConnected(false);
    setConnectionStatus('disconnected');
    setReconnectAttempts(0);
    reconnectAttemptsRef.current = 0;
    setLastError(null);
  }, []);

  // Reset reconnection attempts
  const resetReconnection = useCallback(() => {
    setReconnectAttempts(0);
    reconnectAttemptsRef.current = 0;
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
  }, []);

  // Auto-connect on mount
  useEffect(() => {
    connect();

    // Cleanup on unmount
    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  return {
    ros: rosRef.current,
    isConnected,
    connectionStatus,
    reconnectAttempts,
    lastError,
    connect,
    disconnect,
    resetReconnection,
    maxReconnectAttempts
  };
};
