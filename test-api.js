/**
 * Quick test script for backend API
 */

const BACKEND_URL = 'https://ragbackend-production-629d.up.railway.app';

async function testHealthCheck() {
  console.log('Testing health check...');
  try {
    const response = await fetch(`${BACKEND_URL}/`);
    const data = await response.json();
    console.log('✓ Health check passed:', data);
    return true;
  } catch (error) {
    console.error('✗ Health check failed:', error.message);
    return false;
  }
}

async function testQuery() {
  console.log('\nTesting query endpoint...');
  try {
    const response = await fetch(`${BACKEND_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        text: 'Hello, can you help me?',
      }),
    });

    const data = await response.json();
    console.log('Response status:', response.status);
    console.log('Response data:', JSON.stringify(data, null, 2));

    if (Array.isArray(data) && data.length > 0) {
      if (data[0].error) {
        console.log('✗ Query returned error:', data[0].error);
      } else if (data[0].response) {
        console.log('✓ Query successful:', data[0].response);
      }
    }
  } catch (error) {
    console.error('✗ Query failed:', error.message);
  }
}

async function runTests() {
  console.log('=== Backend API Tests ===\n');
  await testHealthCheck();
  await testQuery();
  console.log('\n=== Tests Complete ===');
}

runTests();
