/**
 * Chatbot API Service
 * Handles communication with the RAG chatbot backend
 */

const BACKEND_URL = 'https://ragbackend-production-629d.up.railway.app';

/**
 * Send a query to the chatbot backend
 * @param {string} query - The user's question or selected text
 * @param {string} conversationId - Optional conversation ID for context
 * @returns {Promise<Object>} Response from the chatbot
 */
export async function sendChatQuery(query, conversationId = null) {
  try {
    const response = await fetch(`${BACKEND_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        text: query,
      }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.error || `HTTP error! status: ${response.status}`);
    }

    const data = await response.json();

    // Handle the response format: [{"response": "..."}, status_code]
    if (Array.isArray(data) && data.length > 0) {
      return {
        answer: data[0].response || data[0].error || 'No response received',
        status: data[1],
        error: data[0].error,
      };
    }

    return data;
  } catch (error) {
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
