/**
 * Root Component - Global wrapper for all pages
 * Injects chatbot and text selection handler
 */

import React from 'react';
import RagChatbot from '../components/RagChatbot';
import TextSelectionHandler from '../components/TextSelectionHandler';

export default function Root({ children }) {
  return (
    <>
      {children}
      <TextSelectionHandler />
      <RagChatbot />
    </>
  );
}
