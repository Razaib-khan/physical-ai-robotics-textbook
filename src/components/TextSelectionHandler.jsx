/**
 * Text Selection Handler Component
 * Displays context menu when text is selected
 */

import React, { useState, useEffect } from 'react';
import './TextSelectionHandler.css';

export default function TextSelectionHandler() {
  const [menuVisible, setMenuVisible] = useState(false);
  const [menuPosition, setMenuPosition] = useState({ x: 0, y: 0 });
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleMouseUp = () => {
      // Small delay to ensure selection is complete
      setTimeout(() => {
        const selection = window.getSelection();
        const text = selection.toString().trim();

        if (text.length > 0) {
          // Get selection bounds for menu positioning
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Position menu below the selection
          setMenuPosition({
            x: rect.left + rect.width / 2,
            y: rect.bottom + window.scrollY + 5,
          });

          setSelectedText(text);
          setMenuVisible(true);
        } else {
          setMenuVisible(false);
        }
      }, 10);
    };

    const handleMouseDown = (e) => {
      // Hide menu when clicking outside
      if (menuVisible && !e.target.closest('.text-selection-menu')) {
        setMenuVisible(false);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
    };
  }, [menuVisible]);

  const handleAskChatbot = (customQuestion = null) => {
    // Dispatch custom event to trigger chatbot
    const event = new CustomEvent('chatbot:sendQuery', {
      detail: {
        text: selectedText,
        question: customQuestion,
      },
    });
    window.dispatchEvent(event);

    // Clear selection and hide menu
    window.getSelection().removeAllRanges();
    setMenuVisible(false);
  };

  if (!menuVisible) {
    return null;
  }

  return (
    <div
      className="text-selection-menu"
      style={{
        left: `${menuPosition.x}px`,
        top: `${menuPosition.y}px`,
      }}
    >
      <div className="text-selection-menu-content">
        <button
          className="text-selection-menu-button"
          onClick={() => handleAskChatbot()}
          title="Ask the chatbot to explain this text"
        >
          <span className="text-selection-menu-icon">💬</span>
          Ask Chatbot
        </button>
        <button
          className="text-selection-menu-button"
          onClick={() => handleAskChatbot('Explain this in simple terms')}
          title="Get a simple explanation"
        >
          <span className="text-selection-menu-icon">📖</span>
          Simplify
        </button>
        <button
          className="text-selection-menu-button"
          onClick={() => handleAskChatbot('What are the key concepts here?')}
          title="Identify key concepts"
        >
          <span className="text-selection-menu-icon">🔑</span>
          Key Concepts
        </button>
      </div>
      <div className="text-selection-menu-arrow"></div>
    </div>
  );
}
