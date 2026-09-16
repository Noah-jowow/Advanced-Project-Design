import { useState, useEffect, useRef, useCallback } from 'react';
import type { SimulationPayload, WsMessage } from '../types';

export function useWebSocket(domain: string) {
  const [status, setStatus] = useState<'CONNECTING' | 'CONNECTED' | 'DISCONNECTED'>('CONNECTING');
  const [data, setData] = useState<SimulationPayload | null>(null);
  const [logs, setLogs] = useState<{timestamp: string, message: string}[]>([]);
  const wsRef = useRef<WebSocket | null>(null);
  const txCountRef = useRef(0);

  const getTimestamp = () => new Date().toLocaleTimeString([], { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });

  const appendLog = useCallback((message: string) => {
    setLogs(prev => [...prev, { timestamp: getTimestamp(), message }].slice(-150));
  }, []);

  useEffect(() => {
    let reconnectTimeout: number;
    let ws: WebSocket;
    
    const connect = () => {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const host = window.location.port === '5173' ? 'localhost:8000' : window.location.host;
      
      ws = new WebSocket(`${protocol}//${host}/ws/stream/${domain}`);
      wsRef.current = ws;

      ws.onopen = () => {
        setStatus('CONNECTED');
        appendLog(`[CLIENT_WS] Link established: /ws/stream/${domain} [Status: ONLINE]`);
      };

      ws.onclose = () => {
        setStatus('DISCONNECTED');
        appendLog(`[CLIENT_WS] Link disconnected from /ws/stream/${domain}. Reconnecting in 3s...`);
        reconnectTimeout = window.setTimeout(connect, 3000);
      };
      
      ws.onmessage = (event) => {
        try {
          const response = JSON.parse(event.data) as WsMessage;
          if (response.type === 'result') {
            setData((prev: SimulationPayload | null) => ({ ...(prev || {}), ...response.data }));
            if (response.data?.status === 'completed') {
              appendLog(`[CLIENT_RX] Completed computation received for domain '${domain}' (${Object.keys(response.data || {}).length} variables)`);
            } else if (response.data?.error) {
              appendLog(`[CLIENT_ERROR] Backend returned error: ${response.data.error}`);
            }
          } else if (response.type === 'log') {
            appendLog(response.message);
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
  }, [domain, appendLog]);

  const sendCommand = useCallback((command: string, payload: SimulationPayload) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ command, data: payload }));
      txCountRef.current += 1;
      
      // Log outgoing command (throttle continuous scan ticks to keep log legible)
      if (command !== 'scan' || txCountRef.current === 1 || txCountRef.current % 16 === 0) {
        const keyList = Object.keys(payload || {}).slice(0, 5).join(', ');
        appendLog(`[CLIENT_TX] Dispatched '${command}' (${domain}) [Params: ${keyList}${Object.keys(payload || {}).length > 5 ? '...' : ''}]`);
      }
    } else {
      console.warn('WebSocket is not open. Cannot send command.');
      appendLog(`[CLIENT_WARN] Failed to transmit '${command}': WebSocket is not connected`);
    }
  }, [domain, appendLog]);

  const clearData = useCallback(() => {
    setData(null);
  }, []);

  return { status, data, logs, sendCommand, clearData };
}

