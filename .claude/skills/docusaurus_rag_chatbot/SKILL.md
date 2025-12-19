# Skill: Docusaurus RAG Chatbot Integration

## Metadata

- **Name**: docusaurus_rag_chatbot
- **Version**: 1.0.0
- **Description**: End-to-end RAG chatbot integration with Docusaurus, Qdrant vector database, Cohere embeddings, and Gemini agents using OpenAI Agents SDK

## Role/Persona

You are an expert in building production-ready RAG (Retrieval-Augmented Generation) chatbots integrated with Docusaurus documentation sites. You specialize in:
- Vector database setup and embedding generation
- OpenAI Agents SDK with Gemini API integration
- FastAPI backend development with caching and rate limiting
- React component integration in Docusaurus
- End-to-end RAG pipeline implementation

## Trigger/Intent

Use this skill when the user requests:
- "Create a RAG chatbot for my Docusaurus site"
- "Integrate a chatbot with vector search"
- "Build a RAG system using Qdrant and Cohere"
- "Add AI-powered Q&A to my documentation"
- "Implement a chatbot using OpenAI Agents SDK with Gemini"
- Any variation involving RAG, Docusaurus, vector databases, or AI agents

## Procedures

### Phase 1: Docusaurus Book/Documentation Setup

**Objective**: Create a Docusaurus site with educational or documentation content.

**Steps**:

1. **Initialize Docusaurus Project**
   ```bash
   npx create-docusaurus@latest frontend classic
   cd frontend
   ```

2. **Configure Docusaurus** (docusaurus.config.js)
   - Set site title, tagline, and URL
   - Configure sidebar navigation
   - Enable KaTeX for math rendering (if needed)
   - Add Mermaid support for diagrams

3. **Create Content Structure**
   - Organize content in `docs/` directory
   - Use hierarchical structure: Module → Chapter → Sections
   - Write content in MDX format
   - Add code examples with syntax highlighting

4. **Deploy Documentation**
   ```bash
   npm run build
   # Deploy to GitHub Pages, Vercel, or Netlify
   ```

**Success Criteria**:
- ✅ Docusaurus site builds without errors
- ✅ Content is accessible and readable
- ✅ Site is deployed and has public URL (e.g., https://username.github.io/project/)

### Phase 2: Vector Database & Embedding Generation

**Objective**: Convert documentation content into vector embeddings and store in Qdrant.

**Steps**:

1. **Setup Qdrant Cloud**
   - Create account at https://qdrant.tech/
   - Create a new cluster
   - Note: Cluster URL and API key
   - Create collection (e.g., "Physical-ai-book-cluster")

2. **Setup Cohere API**
   - Create account at https://cohere.com/
   - Get API key for embeddings
   - Model to use: `embed-english-v3.0` (1024 dimensions)

3. **Create Ingestion Script** (RAGBackend/ingestion.py)
   ```python
   import requests
   import xml.etree.ElementTree as ET
   import trafilatura
   from qdrant_client import QdrantClient
   from qdrant_client.models import VectorParams, Distance, PointStruct
   import cohere
   import os
   from dotenv import load_dotenv

   load_dotenv()

   # Configuration
   SITEMAP_URL = "https://your-site.github.io/sitemap.xml"
   COLLECTION_NAME = "your-collection-name"

   # Initialize clients
   cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
   qdrant = QdrantClient(
       url=os.getenv("QDRANT_URL"),
       api_key=os.getenv("QDRANT_API_KEY")
   )

   # Extract URLs from sitemap
   def get_all_urls(sitemap_url):
       xml = requests.get(sitemap_url).text
       root = ET.fromstring(xml)
       urls = []
       for child in root:
           loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
           if loc_tag is not None:
               urls.append(loc_tag.text)
       return urls

   # Extract text from URL
   def extract_text_from_url(url):
       html = requests.get(url).text
       return trafilatura.extract(html)

   # Chunk text (1200 chars max)
   def chunk_text(text, max_chars=1200):
       chunks = []
       while len(text) > max_chars:
           split_pos = text[:max_chars].rfind(". ")
           if split_pos == -1:
               split_pos = max_chars
           chunks.append(text[:split_pos])
           text = text[split_pos:]
       chunks.append(text)
       return chunks

   # Create embeddings
   def embed(text):
       response = cohere_client.embed(
           model="embed-english-v3.0",
           input_type="search_query",
           texts=[text],
       )
       return response.embeddings[0]

   # Create collection
   def create_collection():
       qdrant.recreate_collection(
           collection_name=COLLECTION_NAME,
           vectors_config=VectorParams(
               size=1024,  # Cohere embed-english-v3.0 dimension
               distance=Distance.COSINE
           )
       )

   # Save chunk to Qdrant
   def save_chunk_to_qdrant(chunk, chunk_id, url):
       vector = embed(chunk)
       qdrant.upsert(
           collection_name=COLLECTION_NAME,
           points=[PointStruct(
               id=chunk_id,
               vector=vector,
               payload={"url": url, "text": chunk, "chunk_id": chunk_id}
           )]
       )

   # Main ingestion pipeline
   def ingest_book():
       urls = get_all_urls(SITEMAP_URL)
       create_collection()
       global_id = 1
       for url in urls:
           text = extract_text_from_url(url)
           if not text:
               continue
           chunks = chunk_text(text)
           for ch in chunks:
               save_chunk_to_qdrant(ch, global_id, url)
               print(f"Saved chunk {global_id}")
               global_id += 1
       print(f"✔️ Ingestion completed! Total chunks: {global_id - 1}")
   ```

4. **Run Ingestion**
   ```bash
   cd RAGBackend
   python ingestion.py
   ```

**Success Criteria**:
- ✅ All documentation pages are crawled
- ✅ Text is extracted and chunked
- ✅ Embeddings are generated and stored in Qdrant
- ✅ Collection is searchable

### Phase 3: Gemini Agent with OpenAI Agents SDK

**Objective**: Create an AI agent using OpenAI Agents SDK with Gemini API that retrieves context from Qdrant.

**Steps**:

1. **Setup Gemini API**
   - Go to https://ai.google.dev/
   - Get Gemini API key
   - Model to use: `gemini-2.0-flash-exp` (via OpenAI-compatible endpoint)

2. **Create Connection Config** (RAGBackend/connection.py)
   ```python
   import os
   from dotenv import load_dotenv
   from agents import OpenAIConfig

   load_dotenv()

   # Validate required environment variable
   gemini_api_key = os.getenv("GEMINI_API_KEY")
   if not gemini_api_key:
       raise ValueError("GEMINI_API_KEY not found in environment variables")

   # OpenAI-compatible configuration for Gemini
   config = OpenAIConfig(
       api_key=gemini_api_key,
       base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
       model="gemini-2.0-flash-exp"
   )
   ```

3. **Create Retrieval Agent** (RAGBackend/agent.py)
   ```python
   from agents import Agent, Runner, set_tracing_disabled, function_tool
   import os
   from dotenv import load_dotenv
   from agents import enable_verbose_stdout_logging
   from connection import config
   import cohere
   from qdrant_client import QdrantClient

   enable_verbose_stdout_logging()
   load_dotenv()
   set_tracing_disabled(disabled=False)

   # Initialize Cohere client
   cohere_client = cohere.Client(os.getenv("COHERE_MODEL_API"))

   # Connect to Qdrant
   qdrant = QdrantClient(
       url=os.getenv("QDRANT_VECTOR_DATABASE_URL_ENDPOINT"),
       api_key=os.getenv("QDRANT_VECTOR_DATABASE_API_KEY")
   )

   embedding_cache = {}

   def get_embedding(text):
       """Get embedding vector from Cohere Embed v3"""
       if text in embedding_cache:
           return embedding_cache[text]

       response = cohere_client.embed(
           model="embed-english-v3.0",
           input_type="search_query",  # Use search_query for queries
           texts=[text],
       )
       embedding = response.embeddings[0]
       embedding_cache[text] = embedding
       return embedding

   @function_tool
   def retrieve(query):
       """Retrieve relevant context from vector database"""
       embedding = get_embedding(query)
       result = qdrant.query_points(
           collection_name="Physical-ai-book-cluster",
           query=embedding,
           limit=5
       )
       return [point.payload["text"] for point in result.points]

   # Create agent with retrieve tool
   agent = Agent(
       name="Assistant",
       instructions="""
   You are an AI tutor for the documentation.
   To answer the user question, first call the tool `retrieve` with the user query.
   Use ONLY the returned content from `retrieve` to answer.
   If the answer is not in the retrieved content, say "I don't know".
   """,
       tools=[retrieve]
   )
   ```

**Success Criteria**:
- ✅ Agent is configured with Gemini API
- ✅ Retrieve tool successfully queries Qdrant
- ✅ Agent generates answers based on retrieved context

### Phase 4: FastAPI Backend with Caching

**Objective**: Create a production-ready FastAPI backend with caching and rate limiting.

**Steps**:

1. **Create FastAPI App** (RAGBackend/main.py)
   ```python
   from fastapi import FastAPI, Request
   from fastapi.middleware.cors import CORSMiddleware
   from fastapi.responses import JSONResponse
   import os
   import time
   import hashlib
   from collections import OrderedDict
   from typing import Dict, Tuple
   from agent import agent
   from agents import Runner
   from connection import config

   # TTL Cache implementation
   class TTLCache:
       def __init__(self, max_size: int = 100, ttl: int = 3600):
           self.cache: OrderedDict[str, Tuple[dict, float]] = OrderedDict()
           self.max_size = max_size
           self.ttl = ttl
           self.hits = 0
           self.misses = 0

       def get(self, key: str) -> dict | None:
           if key not in self.cache:
               self.misses += 1
               return None
           value, timestamp = self.cache[key]
           if time.time() - timestamp > self.ttl:
               del self.cache[key]
               self.misses += 1
               return None
           self.cache.move_to_end(key)
           self.hits += 1
           return value

       def set(self, key: str, value: dict):
           if key in self.cache:
               del self.cache[key]
           self.cache[key] = (value, time.time())
           if len(self.cache) > self.max_size:
               self.cache.popitem(last=False)

   # Rate limiter
   class RateLimiter:
       def __init__(self, requests_per_minute: int = 10):
           self.requests_per_minute = requests_per_minute
           self.user_requests: Dict[str, list] = {}

       def is_allowed(self, client_ip: str) -> bool:
           current_time = time.time()
           if client_ip not in self.user_requests:
               self.user_requests[client_ip] = []
           self.user_requests[client_ip] = [
               req_time for req_time in self.user_requests[client_ip]
               if current_time - req_time < 60
           ]
           if len(self.user_requests[client_ip]) >= self.requests_per_minute:
               return False
           self.user_requests[client_ip].append(current_time)
           return True

   # Initialize
   response_cache = TTLCache(max_size=100, ttl=3600)
   rate_limiter = RateLimiter(requests_per_minute=10)

   app = FastAPI()

   # CORS configuration
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["*"],  # Replace with specific origins in production
       allow_credentials=True,
       allow_methods=["*"],
       allow_headers=["*"],
   )

   @app.get("/")
   async def read_root():
       return {"message": "Welcome to the RAG Backend!"}

   def get_cache_key(query: str) -> str:
       normalized = query.lower().strip()
       normalized = ' '.join(normalized.split())
       if len(normalized) > 200:
           return hashlib.md5(normalized.encode()).hexdigest()
       return normalized

   @app.post("/query")
   async def process_query(query: dict, request: Request):
       user_query = query.get("text")

       # Validate input
       if not user_query or not user_query.strip():
           return JSONResponse(
               status_code=400,
               content={"error": "No query text provided"}
           )

       if len(user_query) > 1000:
           return JSONResponse(
               status_code=400,
               content={"error": "Query too long (max 1000 characters)"}
           )

       # Rate limiting
       client_ip = request.client.host
       if not rate_limiter.is_allowed(client_ip):
           return JSONResponse(
               status_code=429,
               content={"error": "Rate limit exceeded. Please try again later."}
           )

       # Check cache
       cache_key = get_cache_key(user_query)
       cached_response = response_cache.get(cache_key)
       if cached_response:
           print(f"✅ Cache HIT for query: {user_query[:50]}...")
           cached_response["cached"] = True
           return cached_response

       print(f"❌ Cache MISS for query: {user_query[:50]}...")

       try:
           # Run agent
           result = await Runner.run(agent, input=user_query, run_config=config)
           response_text = result.final_output

           response = {"response": response_text, "cached": False}
           response_cache.set(cache_key, response)

           return response
       except Exception as e:
           print(f"Error: {e}")
           return JSONResponse(
               status_code=500,
               content={"error": str(e)[:200]}
           )
   ```

2. **Create Dependencies File** (RAGBackend/pyproject.toml)
   ```toml
   [project]
   name = "rag-backend"
   version = "0.1.0"
   requires-python = ">=3.12"
   dependencies = [
       "cohere>=5.20.0",
       "fastapi>=0.124.0",
       "openai-agents>=0.6.2",
       "qdrant-client>=1.16.1",
       "requests>=2.32.5",
       "trafilatura>=2.0.0",
       "uvicorn>=0.38.0",
       "python-dotenv>=1.0.0",
   ]
   ```

3. **Environment Variables** (.env)
   ```
   GEMINI_API_KEY=your_gemini_api_key
   COHERE_MODEL_API=your_cohere_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_VECTOR_DATABASE_URL_ENDPOINT=https://your-cluster.qdrant.io
   QDRANT_VECTOR_DATABASE_API_KEY=your_qdrant_api_key
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your_qdrant_api_key
   ```

4. **Run Backend**
   ```bash
   cd RAGBackend
   pip install -e .
   uvicorn main:app --reload --port 8000
   ```

5. **Deploy to Railway**
   - Create Railway account
   - Create new project
   - Connect GitHub repository
   - Add environment variables
   - Deploy automatically on push

**Success Criteria**:
- ✅ Backend runs locally without errors
- ✅ /query endpoint accepts requests and returns responses
- ✅ Caching reduces API calls (check logs)
- ✅ Rate limiting prevents spam
- ✅ Backend is deployed on Railway

### Phase 5: React Chatbot Component Integration

**Objective**: Create a React chatbot component and integrate into Docusaurus.

**Steps**:

1. **Create API Service** (frontend/src/services/chatbotApi.js)
   ```javascript
   const BACKEND_URL = 'https://your-backend.railway.app';

   export async function sendChatQuery(text, conversationId = null) {
       const controller = new AbortController();
       const timeoutId = setTimeout(() => controller.abort(), 60000); // 60s timeout

       try {
           const response = await fetch(`${BACKEND_URL}/query`, {
               method: 'POST',
               headers: { 'Content-Type': 'application/json' },
               body: JSON.stringify({ text, conversation_id: conversationId }),
               signal: controller.signal
           });

           clearTimeout(timeoutId);

           if (!response.ok) {
               const errorData = await response.json();
               throw new Error(errorData.error || 'Request failed');
           }

           return await response.json();
       } catch (error) {
           if (error.name === 'AbortError') {
               throw new Error('Request timeout - please try again');
           }
           throw error;
       }
   }

   export async function checkBackendHealth() {
       try {
           const response = await fetch(`${BACKEND_URL}/`);
           return response.ok;
       } catch {
           return false;
       }
   }
   ```

2. **Create Chatbot Component** (frontend/src/components/RagChatbot.jsx)
   ```javascript
   import React, { useState, useEffect, useRef } from 'react';
   import { sendChatQuery, checkBackendHealth } from '../services/chatbotApi';
   import './RagChatbot.css';

   export default function RagChatbot() {
       const [isOpen, setIsOpen] = useState(false);
       const [messages, setMessages] = useState([]);
       const [inputValue, setInputValue] = useState('');
       const [isLoading, setIsLoading] = useState(false);
       const [validationError, setValidationError] = useState(null);
       const MAX_QUERY_LENGTH = 1000;

       const validateQuery = (text) => {
           if (!text || text.trim() === '') {
               return 'Please enter a question';
           }
           if (text.length > MAX_QUERY_LENGTH) {
               return `Query is too long (max ${MAX_QUERY_LENGTH} characters)`;
           }
           return null;
       };

       const handleSendMessage = async () => {
           const validationErrorMessage = validateQuery(inputValue);
           if (validationErrorMessage) {
               setValidationError(validationErrorMessage);
               return;
           }

           setValidationError(null);
           const userMessage = {
               role: 'user',
               content: inputValue,
               timestamp: new Date().toISOString(),
           };

           setMessages(prev => [...prev, userMessage]);
           setInputValue('');
           setIsLoading(true);

           try {
               const response = await sendChatQuery(userMessage.content);
               const botMessage = {
                   role: 'assistant',
                   content: response.error ? response.error : (response.response || 'Sorry, I could not process your request.'),
                   timestamp: new Date().toISOString(),
                   error: !!response.error,
               };
               setMessages(prev => [...prev, botMessage]);
           } catch (error) {
               const errorMessage = {
                   role: 'assistant',
                   content: error.message || 'Failed to connect to the server.',
                   timestamp: new Date().toISOString(),
                   error: true,
               };
               setMessages(prev => [...prev, errorMessage]);
           } finally {
               setIsLoading(false);
           }
       };

       return (
           <div className="chatbot-container">
               {isOpen && (
                   <div className="chatbot-window">
                       <div className="chatbot-header">
                           <h3>RAG Chatbot</h3>
                           <button onClick={() => setIsOpen(false)}>✕</button>
                       </div>
                       <div className="chatbot-messages">
                           {messages.map((msg, index) => (
                               <div key={index} className={`message-${msg.role}`}>
                                   {msg.content}
                               </div>
                           ))}
                           {isLoading && <div>Loading...</div>}
                       </div>
                       {validationError && <div className="error">{validationError}</div>}
                       <div className="chatbot-input">
                           <textarea
                               value={inputValue}
                               onChange={(e) => setInputValue(e.target.value)}
                               placeholder="Ask a question..."
                               maxLength={MAX_QUERY_LENGTH}
                           />
                           <button onClick={handleSendMessage} disabled={isLoading}>
                               Send
                           </button>
                       </div>
                   </div>
               )}
               <button onClick={() => setIsOpen(!isOpen)}>
                   {isOpen ? '✕' : '💬'}
               </button>
           </div>
       );
   }
   ```

3. **Add Chatbot to Docusaurus** (frontend/src/theme/Root.jsx)
   ```javascript
   import React from 'react';
   import RagChatbot from '../components/RagChatbot';

   export default function Root({ children }) {
       return (
           <>
               {children}
               <RagChatbot />
           </>
       );
   }
   ```

4. **Build and Deploy**
   ```bash
   npm run build
   # Deploy to GitHub Pages / Vercel
   ```

**Success Criteria**:
- ✅ Chatbot appears on all pages
- ✅ Users can send queries
- ✅ Responses are displayed correctly
- ✅ Input validation prevents invalid queries
- ✅ Timeout handling works

### Phase 6: Testing & Documentation

**Steps**:

1. **Manual Testing**
   - Test end-to-end query flow
   - Verify caching (check backend logs for cache hits)
   - Test rate limiting (send rapid requests)
   - Test timeout (throttle network)
   - Test error handling (disconnect internet)

2. **Document Setup**
   - Create README with setup instructions
   - Document environment variables
   - Add architecture diagram
   - Include troubleshooting guide

3. **Optional: Use Context7 MCP for Documentation**
   - Install Context7 MCP server
   - Use for getting latest documentation of libraries
   - Reference: https://github.com/punkpeye/context7

**Success Criteria**:
- ✅ All test scenarios pass
- ✅ Documentation is complete and accurate

## Examples

### Example 1: Creating a Physics Textbook RAG Chatbot

**User Request**: "I want to create a RAG chatbot for my physics textbook hosted on Docusaurus"

**Assistant Response**:
```
I'll help you create a RAG chatbot for your physics textbook. Here's the plan:

**Phase 1: Docusaurus Setup**
- Initialize Docusaurus with `npx create-docusaurus@latest`
- Organize physics content by chapters
- Deploy to GitHub Pages

**Phase 2: Vector Database**
- Setup Qdrant cloud cluster
- Get Cohere API key for embeddings
- Create ingestion script to convert textbook to vectors

**Phase 3: Gemini Agent**
- Setup Gemini API
- Create agent with retrieve tool using OpenAI Agents SDK
- Configure to answer only from textbook content

**Phase 4: FastAPI Backend**
- Create backend with caching (reduce API costs)
- Add rate limiting
- Deploy to Railway

**Phase 5: React Component**
- Create chatbot UI
- Integrate into Docusaurus
- Add input validation and error handling

Let's start with Phase 1. Do you already have a Docusaurus site, or should I help you set one up?
```

### Example 2: Debugging Embedding Issues

**User Request**: "My embeddings aren't working - Qdrant returns no results"

**Assistant Response**:
```
Let me help you debug. Common issues:

1. **Check embedding model consistency**
   - Ingestion uses: embed-english-v3.0 with input_type="search_query"
   - Query must use: SAME model with SAME input_type

2. **Verify collection dimensions**
   - Cohere embed-english-v3.0 = 1024 dimensions
   - Check: qdrant.get_collection("your-collection")

3. **Test retrieval directly**
   ```python
   from agent import get_embedding
   from qdrant_client import QdrantClient

   qdrant = QdrantClient(url=..., api_key=...)
   test_query = "What is physics?"
   embedding = get_embedding(test_query)

   results = qdrant.query_points(
       collection_name="your-collection",
       query=embedding,
       limit=5
   )
   print(f"Found {len(results.points)} results")
   for point in results.points:
       print(point.payload["text"][:100])
   ```

Which step should we check first?
```

## Self-Check/Validation

Before completing the skill execution, verify:

### Phase 1: Docusaurus
- [ ] Site builds without errors (`npm run build`)
- [ ] Content is organized logically
- [ ] Site is deployed and accessible via URL
- [ ] Sitemap.xml is generated

### Phase 2: Embeddings
- [ ] Qdrant collection exists and is populated
- [ ] Embedding dimensions match (1024 for Cohere embed-english-v3.0)
- [ ] Test query returns relevant results
- [ ] All pages were successfully ingested

### Phase 3: Agent
- [ ] Gemini API key is valid
- [ ] Agent can be instantiated without errors
- [ ] Retrieve tool successfully queries Qdrant
- [ ] Agent follows instructions (answers only from retrieved content)

### Phase 4: Backend
- [ ] Backend starts without errors
- [ ] /query endpoint responds correctly
- [ ] Cache reduces repeat queries (check logs)
- [ ] Rate limiting works (test with rapid requests)
- [ ] Environment variables are validated on startup
- [ ] Backend is deployed and accessible

### Phase 5: Frontend
- [ ] Chatbot renders on page
- [ ] Input validation prevents empty/long queries
- [ ] Timeout handling works (test with network throttling)
- [ ] Error messages are user-friendly
- [ ] Frontend communicates with deployed backend

### Phase 6: End-to-End
- [ ] User can ask a question and get answer within 5 seconds
- [ ] Answer is relevant and from textbook content
- [ ] Subsequent identical queries use cache
- [ ] "I don't know" is returned when content not found
- [ ] All error scenarios display friendly messages

### Documentation
- [ ] README contains all setup steps
- [ ] Environment variables are documented
- [ ] API endpoints are documented
- [ ] Architecture is explained

**If all checks pass**: ✅ RAG chatbot is production-ready!

**If checks fail**: Identify failing phase and troubleshoot using Examples section.
