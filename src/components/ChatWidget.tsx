import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

interface Citation {
  chapter_title: string;
  chapter_slug: string;
  chapter_file: string;
}

interface Message {
  id?: number;
  user_message?: string;
  bot_response?: string;
  answer?: string;
  citations?: Citation[];
  is_in_scope?: boolean;
  isUser?: boolean;
  text?: string;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');

    // Add user message to UI
    setMessages((prev) => [...prev, { isUser: true, text: userMessage }]);
    setIsLoading(true);

    // Detect language from URL path (e.g., /ur/docs/...)
    const currentLanguage = window.location.pathname.startsWith('/ur') ? 'ur' : 'en';

    // Simulate processing delay for UI demonstration
    setTimeout(() => {
      // Add bot response indicating backend is not available
      setMessages((prev) => [
        ...prev,
        {
          isUser: false,
          text: currentLanguage === 'ur'
            ? '⚠️ معذرت، RAG چیٹ بوٹ بیک اینڈ دستیاب نہیں ہے۔ یہ UI صرف مظاہرے کے لیے ہے۔'
            : '⚠️ Sorry, the RAG chatbot backend is not available. This UI is for demonstration purposes only.',
          citations: [],
        },
      ]);
      setIsLoading(false);
    }, 1000);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Chat Button */}
      {!isOpen && (
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬 Ask AI
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div>
              <h3>AI Textbook Assistant</h3>
              <p>Ask questions about Physical AI & Robotics</p>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I'm your AI assistant for this textbook.</p>
                <p>Ask me anything about Physical AI and Humanoid Robotics!</p>
                <div className={styles.exampleQuestions}>
                  <p><strong>Example questions:</strong></p>
                  <button onClick={() => setInput("What is the Zero Moment Point?")}>
                    What is the Zero Moment Point?
                  </button>
                  <button onClick={() => setInput("How do humanoid robots balance?")}>
                    How do humanoid robots balance?
                  </button>
                  <button onClick={() => setInput("What sensors do robots use?")}>
                    What sensors do robots use?
                  </button>
                </div>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={msg.isUser ? styles.userMessage : styles.botMessage}
              >
                <div className={styles.messageContent}>
                  <p>{msg.text}</p>
                  {msg.citations && msg.citations.length > 0 && (
                    <div className={styles.citations}>
                      <p><strong>📚 Sources:</strong></p>
                      {msg.citations.map((citation, cidx) => (
                        <a
                          key={cidx}
                          href={`/docs/${citation.chapter_file.replace('.md', '')}`}
                          className={styles.citationLink}
                        >
                          {citation.chapter_title}
                        </a>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={styles.botMessage}>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.chatInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              disabled={isLoading}
              rows={2}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !input.trim()}
              aria-label="Send message"
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
