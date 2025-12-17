/**
 * RAG Chatbot API Integration for Frontend
 *
 * Usage in your React/Next.js app:
 * import { ragAPI } from './frontend-integration';
 * const response = await ragAPI.ask("What are IMU sensors?");
 */

// Auto-detect backend URL
const getAPIUrl = () => {
  // Check if running in development or production
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;

    // Development (localhost)
    if (hostname === 'localhost' || hostname === '127.0.0.1') {
      return 'http://localhost:8000';
    }

    // Production (Vercel deployment)
    return 'http://localhost:8000'; // Update this with your deployed backend URL
  }

  return 'http://localhost:8000';
};

const API_URL = getAPIUrl();

/**
 * RAG Chatbot API Client
 */
export const ragAPI = {
  /**
   * Ask a question to the RAG chatbot
   * @param {string} query - User's question
   * @param {number} topK - Number of chunks to retrieve (default: 3)
   * @returns {Promise<Object>} Response with answer and citations
   */
  async ask(query, topK = 3) {
    try {
      const response = await fetch(`${API_URL}/api/v1/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
          top_k: topK
        })
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || 'Failed to get response');
      }

      return await response.json();
    } catch (error) {
      console.error('RAG API Error:', error);
      throw error;
    }
  },

  /**
   * Check backend health status
   * @returns {Promise<Object>} Health status of all services
   */
  async checkHealth() {
    try {
      const response = await fetch(`${API_URL}/api/v1/health`);
      return await response.json();
    } catch (error) {
      console.error('Health check failed:', error);
      return { status: 'down', error: error.message };
    }
  },

  /**
   * Get query statistics
   * @param {string} startDate - Start date (ISO format, optional)
   * @param {string} endDate - End date (ISO format, optional)
   * @returns {Promise<Object>} Query statistics
   */
  async getStats(startDate, endDate) {
    try {
      const params = new URLSearchParams();
      if (startDate) params.append('start_date', startDate);
      if (endDate) params.append('end_date', endDate);

      const response = await fetch(
        `${API_URL}/api/v1/analytics/stats?${params.toString()}`
      );
      return await response.json();
    } catch (error) {
      console.error('Stats fetch failed:', error);
      throw error;
    }
  },

  /**
   * Get top errors
   * @param {number} limit - Number of errors to fetch (default: 10)
   * @returns {Promise<Object>} Top errors
   */
  async getTopErrors(limit = 10) {
    try {
      const response = await fetch(
        `${API_URL}/api/v1/analytics/errors?limit=${limit}`
      );
      return await response.json();
    } catch (error) {
      console.error('Errors fetch failed:', error);
      throw error;
    }
  }
};

/**
 * React Hook for RAG Chatbot
 *
 * Usage:
 * const { ask, loading, error, response } = useRAGChat();
 */
export const useRAGChat = () => {
  const [loading, setLoading] = React.useState(false);
  const [error, setError] = React.useState(null);
  const [response, setResponse] = React.useState(null);

  const ask = async (query, topK = 3) => {
    setLoading(true);
    setError(null);

    try {
      const result = await ragAPI.ask(query, topK);
      setResponse(result);
      return result;
    } catch (err) {
      setError(err.message);
      throw err;
    } finally {
      setLoading(false);
    }
  };

  return { ask, loading, error, response };
};

/**
 * Example usage in React component:
 *
 * function ChatInterface() {
 *   const { ask, loading, error, response } = useRAGChat();
 *   const [query, setQuery] = useState('');
 *
 *   const handleSubmit = async (e) => {
 *     e.preventDefault();
 *     await ask(query);
 *   };
 *
 *   return (
 *     <div>
 *       <form onSubmit={handleSubmit}>
 *         <input
 *           value={query}
 *           onChange={(e) => setQuery(e.target.value)}
 *           placeholder="Ask a question..."
 *         />
 *         <button type="submit" disabled={loading}>
 *           {loading ? 'Loading...' : 'Ask'}
 *         </button>
 *       </form>
 *
 *       {error && <div className="error">{error}</div>}
 *
 *       {response && (
 *         <div className="response">
 *           <p>{response.answer}</p>
 *           <div className="citations">
 *             {response.citations.map((citation, idx) => (
 *               <span key={idx}>
 *                 Chapter {citation.chapter}
 *                 {citation.section && `, ${citation.section}`}
 *                 {citation.page && `, p. ${citation.page}`}
 *               </span>
 *             ))}
 *           </div>
 *           <small>Latency: {response.latency_ms}ms</small>
 *         </div>
 *       )}
 *     </div>
 *   );
 * }
 */
