/**
 * RAG Chatbot Component
 * Floating chatbot interface with text selection support
 */

import React, { useState, useEffect, useRef } from 'react';
import { sendChatQuery, checkBackendHealth } from '../services/chatbotApi';
import './RagChatbot.css';

export default function RagChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const [backendHealthy, setBackendHealthy] = useState(true);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Check backend health on mount
  useEffect(() => {
    const checkHealth = async () => {
      const healthy = await checkBackendHealth();
      setBackendHealthy(healthy);
    };
    checkHealth();
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Listen for text selection events
  useEffect(() => {
    const handleTextSelection = (event) => {
      if (event.detail && event.detail.text) {
        setIsOpen(true);
        const questionText = event.detail.question || `Explain this: ${event.detail.text}`;
        handleSendMessage(questionText, event.detail.text);
      }
    };

    window.addEventListener('chatbot:sendQuery', handleTextSelection);
    return () => {
      window.removeEventListener('chatbot:sendQuery', handleTextSelection);
    };
  }, [messages, conversationId]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (messageText, selectedText = null) => {
    // Get the message content from either parameter or input field
    const messageContent = messageText || inputValue;

    // Check if we have valid content
    if (!messageContent || (!messageContent.trim() && !selectedText)) return;

    const userMessage = {
      role: 'user',
      content: messageContent,
      timestamp: new Date().toISOString(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await sendChatQuery(userMessage.content, conversationId);

      // Update conversation ID if provided
      if (response.conversation_id) {
        setConversationId(response.conversation_id);
      }

      // Check if the response contains an error
      const hasError = response.error || response.status >= 400;

      // Transform generic errors into user-friendly messages
      let errorContent = '';
      if (hasError) {
        errorContent = '⚠️ LLM API quota has been exceeded.\n\n' +
                      'Please try again later. Thank you for your patience! 🙏';
      }

      const botMessage = {
        role: 'assistant',
        content: hasError ? errorContent : (response.answer || response.response || 'Sorry, I could not process your request.'),
        timestamp: new Date().toISOString(),
        error: hasError,
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        role: 'assistant',
        content: '⚠️ LLM API quota has been exceeded.\n\n' +
                'Please try again later. Thank you for your patience! 🙏',
        timestamp: new Date().toISOString(),
        error: true,
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setConversationId(null);
  };

  if (!backendHealthy) {
    return (
      <div className="chatbot-container">
        <button
          className="chatbot-button chatbot-button-error"
          onClick={toggleChat}
          aria-label="Chatbot (Backend Unavailable)"
        >
          <span className="chatbot-icon">⚠️</span>
        </button>
      </div>
    );
  }

  return (
    <div className="chatbot-container">
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>RAG Chatbot Assistant</h3>
            <div className="chatbot-header-actions">
              <button
                onClick={clearChat}
                className="chatbot-clear-button"
                aria-label="Clear chat"
                title="Clear chat"
              >
                🗑️
              </button>
              <button
                onClick={toggleChat}
                className="chatbot-close-button"
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 && (
              <div className="chatbot-welcome">
                <p>👋 Welcome! I'm your RAG-powered assistant.</p>
                <p>Ask me anything or select text on the page to get explanations!</p>
              </div>
            )}
            {messages.map((msg, index) => (
              <div
                key={index}
                className={`chatbot-message chatbot-message-${msg.role} ${msg.error ? 'chatbot-message-error' : ''}`}
              >
                <div className="chatbot-message-content">
                  {msg.content}
                </div>
                <div className="chatbot-message-timestamp">
                  {new Date(msg.timestamp).toLocaleTimeString()}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="chatbot-message chatbot-message-assistant">
                <div className="chatbot-message-content chatbot-loading">
                  <span className="chatbot-loading-dot">●</span>
                  <span className="chatbot-loading-dot">●</span>
                  <span className="chatbot-loading-dot">●</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-container">
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message... (Press Enter to send)"
              className="chatbot-input"
              rows="2"
              disabled={isLoading}
            />
            <button
              onClick={() => handleSendMessage()}
              disabled={isLoading || !inputValue.trim()}
              className="chatbot-send-button"
              aria-label="Send message"
            >
              ➤
            </button>
          </div>
        </div>
      )}

      <button
        className={`chatbot-button ${isOpen ? 'chatbot-button-open' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chatbot' : 'Open chatbot'}
      >
        <span className="chatbot-icon">{isOpen ? '✕' : '💬'}</span>
      </button>
    </div>
  );
}
