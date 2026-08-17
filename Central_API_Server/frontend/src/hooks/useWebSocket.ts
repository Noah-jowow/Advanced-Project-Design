import { useState, useEffect, useRef, useCallback } from 'react';

export function useWebSocket(domain: string) {
  const [status, setStatus] = useState<'CONNECTING' | 'CONNECTED' | 'DISCONNECTED'>('CONNECTING');
  const [data, setData] = useState<any>(null);
  const [logs, setLogs] = useState<{timestamp: string, message: string}[]>([]);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    let reconnectTimeout: number;
    let ws: WebSocket;
    
    const connect = () => {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const host = window.location.port === '5173' ? 'localhost:8000' : window.location.host;
      
      ws = new WebSocket(`${protocol}//${host}/ws/stream/${domain}`);
      wsRef.current = ws;

      ws.onopen = () => setStatus('CONNECTED');
      ws.onclose = () => {
        setStatus('DISCONNECTED');
        // Auto-reconnect after 3 seconds
        reconnectTimeout = window.setTimeout(connect, 3000);
      };
      
      ws.onmessage = (event) => {
        try {
          const response = JSON.parse(event.data);
          if (response.type === 'result') {
            setData((prev: any) => ({ ...prev, ...response.data }));
          } else if (response.type === 'log') {
            setLogs(prev => [...prev, { 
              timestamp: new Date().toLocaleTimeString([], { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' }),
              message: response.message
            }].slice(-100)); // Keep last 100 logs
          }
        } catch (e) {
          console.error('Failed to parse WebSocket message', e);
        }
      };
    };

    connect();

    return () => {
      clearTimeout(reconnectTimeout);
      if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
        ws.close();
      }
    };
  }, [domain]);

  const sendCommand = useCallback((command: string, payload: any) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ command, data: payload }));
    } else {
      console.warn('WebSocket is not open. Cannot send command.');
    }
  }, []);

  return { status, data, logs, sendCommand };
}
