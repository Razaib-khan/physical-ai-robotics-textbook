/**
 * Chatbot Test Page
 * Diagnostic page to test chatbot integration
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { sendChatQuery, checkBackendHealth } from '../services/chatbotApi';

export default function ChatbotTest() {
  const [backendStatus, setBackendStatus] = useState('Checking...');
  const [testQuery, setTestQuery] = useState('What is robotics?');
  const [testResult, setTestResult] = useState(null);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    checkHealth();
  }, []);

  const checkHealth = async () => {
    const healthy = await checkBackendHealth();
    setBackendStatus(healthy ? '✅ Healthy' : '❌ Unhealthy');
  };

  const runTest = async () => {
    setLoading(true);
    setTestResult(null);
    try {
      const response = await sendChatQuery(testQuery);
      setTestResult({
        success: true,
        data: response,
      });
    } catch (error) {
      setTestResult({
        success: false,
        error: error.message,
      });
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Chatbot Test" description="Test the RAG chatbot integration">
      <div style={{ padding: '2rem', maxWidth: '800px', margin: '0 auto' }}>
        <h1>🧪 Chatbot Integration Test</h1>

        <div style={{ marginBottom: '2rem', padding: '1rem', background: '#f5f5f5', borderRadius: '8px' }}>
          <h2>Backend Status</h2>
          <p style={{ fontSize: '1.5rem' }}>{backendStatus}</p>
          <button onClick={checkHealth} style={{ padding: '0.5rem 1rem', cursor: 'pointer' }}>
            Recheck
          </button>
        </div>

        <div style={{ marginBottom: '2rem', padding: '1rem', background: '#f5f5f5', borderRadius: '8px' }}>
          <h2>Test Query</h2>
          <input
            type="text"
            value={testQuery}
            onChange={(e) => setTestQuery(e.target.value)}
            style={{
              width: '100%',
              padding: '0.5rem',
              fontSize: '1rem',
              marginBottom: '1rem',
              borderRadius: '4px',
              border: '1px solid #ccc',
            }}
          />
          <button
            onClick={runTest}
            disabled={loading}
            style={{
              padding: '0.5rem 1rem',
              cursor: loading ? 'not-allowed' : 'pointer',
              background: loading ? '#ccc' : '#667eea',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
            }}
          >
            {loading ? 'Testing...' : 'Send Test Query'}
          </button>
        </div>

        {testResult && (
          <div
            style={{
              padding: '1rem',
              background: testResult.success ? '#e8f5e9' : '#ffebee',
              borderRadius: '8px',
              marginBottom: '2rem',
            }}
          >
            <h2>{testResult.success ? '✅ Success' : '❌ Error'}</h2>
            <pre
              style={{
                background: 'white',
                padding: '1rem',
                borderRadius: '4px',
                overflow: 'auto',
                fontSize: '0.875rem',
              }}
            >
              {JSON.stringify(testResult.success ? testResult.data : { error: testResult.error }, null, 2)}
            </pre>
          </div>
        )}

        <div style={{ padding: '1rem', background: '#fff3cd', borderRadius: '8px' }}>
          <h2>📝 Testing Checklist</h2>
          <ul>
            <li>Check if the purple chat button (💬) appears in bottom-right corner</li>
            <li>Click the chat button to open the chat window</li>
            <li>Try sending a message in the chat</li>
            <li>Select text on this page to see the context menu</li>
            <li>Check browser console (F12) for any errors</li>
            <li>Check Network tab for API requests</li>
          </ul>
        </div>

        <div style={{ marginTop: '2rem', padding: '1rem', background: '#e3f2fd', borderRadius: '8px' }}>
          <h2>🔧 Configuration</h2>
          <p><strong>Backend URL:</strong> https://ragbackend-production-629d.up.railway.app</p>
          <p><strong>Endpoint:</strong> /query</p>
          <p><strong>Method:</strong> POST</p>
          <p><strong>Request Format:</strong> {`{ "text": "your query" }`}</p>
        </div>
      </div>
    </Layout>
  );
}
