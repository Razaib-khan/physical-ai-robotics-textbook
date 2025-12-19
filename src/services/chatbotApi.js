/**
 * Chatbot API Service
 * Handles communication with the RAG chatbot backend
 */

const BACKEND_URL = 'https://ragbackend-production-629d.up.railway.app';
const TIMEOUT_MS = 60000; // 60 seconds

/**
 * Create a timeout promise using AbortController
 * @param {number} ms - Timeout in milliseconds
 * @returns {Object} Object with promise and abort controller
 */
function createTimeout(ms) {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), ms);

  return {
    signal: controller.signal,
    cleanup: () => clearTimeout(timeoutId),
  };
}

/**
 * Send a query to the chatbot backend
 * @param {string} query - The user's question or selected text
 * @param {string} conversationId - Optional conversation ID for context
 * @returns {Promise<Object>} Response from the chatbot
 */
export async function sendChatQuery(query, conversationId = null) {
  const { signal, cleanup } = createTimeout(TIMEOUT_MS);

  try {
    const response = await fetch(`${BACKEND_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        text: query,
      }),
      signal, // Add AbortController signal
    });

    // Cleanup timeout after receiving response
    cleanup();

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.error || `HTTP error! status: ${response.status}`);
    }

    const data = await response.json();

    // Backend returns single object format: {"response": "..."}
    return {
      answer: data.response || 'No response received',
      error: null,
    };
  } catch (error) {
    // Cleanup timeout
    cleanup();

    // Check if error is due to timeout (AbortError)
    if (error.name === 'AbortError') {
      throw new Error('Request timed out after 60 seconds. Please try again.');
    }

    console.error('Error sending chat query:', error);
    throw error;
  }
}

/**
 * Send a query with selected text context
 * @param {string} selectedText - The text selected by the user
 * @param {string} question - Optional custom question about the selected text
 * @param {string} conversationId - Optional conversation ID for context
 * @returns {Promise<Object>} Response from the chatbot
 */
export async function sendTextSelectionQuery(selectedText, question = null, conversationId = null) {
  const query = question
    ? `${question}\n\nContext: ${selectedText}`
    : `Explain this: ${selectedText}`;

  return sendChatQuery(query, conversationId);
}

/**
 * Check if the backend is healthy
 * @returns {Promise<boolean>} True if backend is healthy
 */
export async function checkBackendHealth() {
  try {
    const response = await fetch(`${BACKEND_URL}/`, {
      method: 'GET',
    });

    if (!response.ok) {
      return false;
    }

    const data = await response.json();
    // Check if we get the welcome message
    return data && data.message === 'Welcome to the RAG Backend!';
  } catch (error) {
    console.error('Backend health check failed:', error);
    return false;
  }
}
