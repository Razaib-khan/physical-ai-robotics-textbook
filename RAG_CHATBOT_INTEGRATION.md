# RAG Chatbot Integration Documentation

## Overview

This document describes the integration of a RAG (Retrieval-Augmented Generation) chatbot with the Docusaurus-based Physical AI & Humanoid Robotics Textbook. The chatbot is connected to a backend at `https://ragbackend-production-629d.up.railway.app` and provides an interactive Q&A experience with text selection functionality.

## Features

### 1. Floating Chatbot Interface
- **Floating Button**: A persistent chat button in the bottom-right corner of every page
- **Expandable Chat Window**: Click the button to open a full chat interface
- **Conversation History**: Messages are preserved during the session
- **Backend Health Check**: Visual indicator if the backend is unavailable

### 2. Text Selection Integration
- **Select Text**: Highlight any text on the page to trigger a context menu
- **Quick Actions**:
  - **Ask Chatbot**: Send the selected text directly to the chatbot
  - **Simplify**: Get a simple explanation of the selected text
  - **Key Concepts**: Extract key concepts from the selected text
- **Seamless Integration**: Selected text is automatically sent as context to the chatbot

### 3. Responsive Design
- Mobile-friendly interface that adapts to different screen sizes
- Dark mode support that matches the Docusaurus theme
- Smooth animations and transitions

## File Structure

```
src/
├── components/
│   ├── RagChatbot.jsx              # Main chatbot component
│   ├── RagChatbot.css              # Chatbot styles
│   ├── TextSelectionHandler.jsx   # Text selection menu component
│   └── TextSelectionHandler.css   # Text selection menu styles
├── services/
│   └── chatbotApi.js              # API service for backend communication
└── theme/
    └── Root.jsx                    # Global wrapper that injects components
```

## Component Details

### RagChatbot.jsx
The main chatbot component that provides:
- Floating chat button
- Chat window with message history
- Input field for user queries
- Integration with backend API
- Event listener for text selection queries
- Backend health monitoring

### TextSelectionHandler.jsx
Handles text selection and displays context menu:
- Detects text selection on the page
- Positions context menu near selected text
- Provides quick action buttons
- Dispatches events to the chatbot component

### chatbotApi.js
API service layer that handles:
- HTTP requests to the backend
- Error handling
- Query formatting
- Backend health checks

### Root.jsx
Global wrapper component that:
- Injects chatbot and text selection handler on every page
- Ensures components are loaded once globally
- Provides consistent experience across all pages

## API Integration

### Backend Endpoint
**Base URL**: `https://ragbackend-production-629d.up.railway.app`

### API Methods

#### Send Chat Query
```javascript
POST /api/chat
Content-Type: application/json

{
  "query": "User's question or selected text",
  "conversation_id": "optional-conversation-id"
}
```

**Response:**
```javascript
{
  "answer": "Chatbot response",
  "conversation_id": "conversation-identifier"
}
```

#### Health Check
```javascript
GET /health
```

Returns HTTP 200 if backend is healthy.

## Usage

### For Users

#### Using the Chatbot
1. Click the purple chat button (💬) in the bottom-right corner
2. Type your question in the input field
3. Press Enter or click the send button (➤)
4. View the chatbot's response in the conversation history
5. Continue the conversation with follow-up questions

#### Using Text Selection
1. Highlight any text on the page
2. A context menu will appear with three options:
   - **Ask Chatbot**: General explanation
   - **Simplify**: Get a simplified explanation
   - **Key Concepts**: Extract key concepts
3. Click any option to send the query to the chatbot
4. The chat window will open automatically with the response

#### Clearing Chat History
- Click the trash icon (🗑️) in the chat header to clear all messages

### For Developers

#### Running in Development
```bash
npm start
```

The chatbot will be available on all pages automatically.

#### Building for Production
```bash
npm run build
```

The chatbot components will be included in the production build.

#### Testing the Integration
1. Start the development server: `npm start`
2. Navigate to any page in the documentation
3. Test the floating chat button
4. Test text selection functionality
5. Verify dark mode compatibility by switching themes

## Customization

### Styling
All styles are contained in CSS files that support both light and dark modes:
- `RagChatbot.css`: Chatbot appearance and animations
- `TextSelectionHandler.css`: Context menu appearance

### Colors
The chatbot uses a purple gradient theme that can be customized:
```css
/* Primary gradient */
background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
```

### Backend URL
To change the backend URL, edit `src/services/chatbotApi.js`:
```javascript
const BACKEND_URL = 'https://your-backend-url.com';
```

## Browser Compatibility

- **Chrome**: Full support
- **Firefox**: Full support
- **Safari**: Full support
- **Edge**: Full support
- **Mobile browsers**: Responsive design with touch support

## Troubleshooting

### Chatbot Button Shows Warning (⚠️)
- The backend is unavailable or unreachable
- Check your internet connection
- Verify the backend URL is correct
- Contact the backend administrator

### Text Selection Menu Not Appearing
- Make sure you're selecting text (not just clicking)
- The menu appears after mouse-up event
- Try selecting text again
- Check browser console for errors

### Messages Not Sending
- Verify backend is healthy (button should show 💬, not ⚠️)
- Check browser console for API errors
- Ensure input field is not empty

## Security Considerations

- All API requests use HTTPS
- No sensitive data is stored in localStorage
- Conversation IDs are session-based
- CORS headers must be configured on the backend

## Performance

- Components are lazy-loaded when needed
- Chat history is maintained in memory (cleared on page refresh)
- API requests include proper error handling and timeouts
- Minimal impact on page load performance

## Future Enhancements

Potential improvements:
- Persist conversation history across sessions
- Add voice input support
- Support for file attachments
- Export conversation history
- Multilingual support
- Typing indicators
- Read receipts
- Suggested questions based on current page content

## Support

For issues or questions about the chatbot integration:
1. Check this documentation
2. Review browser console for errors
3. Test backend health endpoint
4. Contact the development team

## License

This integration follows the same license as the main Physical AI & Humanoid Robotics Textbook project.

---

**Last Updated**: December 8, 2025
**Version**: 1.0.0
**Maintainer**: Development Team
