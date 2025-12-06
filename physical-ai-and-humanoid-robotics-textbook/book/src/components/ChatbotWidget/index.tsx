import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  createdAt: Date;
}

export default function ChatbotWidget() {
  const { siteConfig: { customFields } } = useDocusaurusContext();
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([
    {
      id: '1',
      text: "Hello! I'm your Physical AI & Robotics tutor. How can I help you with the book content?",
      sender: 'bot',
      createdAt: new Date()
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement | null>(null);

  const API_BASE_URL = customFields.API_BASE_URL as string;

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection()?.toString().trim();
      if (selection) {
        setSelectedText(selection);
        setIsChatOpen(true);
      }
    };
    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, []);

  const handleSendMessage = async () => {
    if (!input.trim() && !selectedText.trim()) return;

    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      text: input || `Question about: "${selectedText}"`,
      sender: 'user',
      createdAt: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const endpoint = selectedText ? `${API_BASE_URL}/query_selected_text` : `${API_BASE_URL}/chat`;
      const body = selectedText
        ? JSON.stringify({ selected_text: selectedText, user_question: input })
        : JSON.stringify({ query: input });

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body,
      });

      const data = await response.json();

      const botMessage: ChatMessage = {
        id: Date.now().toString(),
        text: data.response,
        sender: 'bot',
        createdAt: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage: ChatMessage = {
        id: Date.now().toString(),
        text: 'Connection error. Try again later.',
        sender: 'bot',
        createdAt: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText('');
    }
  };

  const toggleChat = () => setIsChatOpen(!isChatOpen);

  if (isChatOpen) {
    // Full screen chat interface
    return (
      <div className="fixed inset-0 z-[9999] bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white">
        {/* Header */}
        <div className="p-6 bg-gradient-to-r from-purple-800 to-blue-800 border-b border-white/20 flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <div className="w-12 h-12 rounded-full bg-gradient-to-r from-cyan-400 to-blue-500 flex items-center justify-center shadow-lg">
              <span className="text-white font-bold text-sm">AI</span>
            </div>
            <div>
              <h1 className="font-bold text-2xl">Physical AI Tutor</h1>
              <p className="text-purple-200 text-sm">Expert in Robotics & AI</p>
            </div>
          </div>
          <button
            onClick={() => setIsChatOpen(false)}
            className="text-white hover:text-gray-300 text-3xl font-bold transition-colors p-2"
          >
            ✕
          </button>
        </div>

        {/* Messages Area */}
        <div className="flex-1 flex flex-col h-[calc(100vh-150px)]">
          <div className="flex-1 p-6 overflow-y-auto space-y-6 custom-scrollbar">
            {messages.map((msg) => (
              <div key={msg.id} className={`flex ${msg.sender === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`flex items-start gap-4 max-w-[80%] ${msg.sender === 'user' ? 'flex-row-reverse' : 'flex-row'}`}>

                  {/* Avatar */}
                  {msg.sender === 'bot' && (
                    <div className="w-10 h-10 rounded-full bg-gradient-to-r from-cyan-400 to-blue-500 flex items-center justify-center shadow-lg flex-shrink-0">
                      <span className="text-white text-xs font-bold">AI</span>
                    </div>
                  )}

                  {/* Message Bubble */}
                  <div className={`rounded-2xl px-5 py-4 ${
                    msg.sender === 'user'
                      ? 'bg-gradient-to-r from-purple-600 to-blue-600 text-white rounded-br-md'
                      : 'bg-white/10 text-white rounded-bl-md border border-white/20 backdrop-blur-sm'
                  }`}>
                    <p className="text-base whitespace-pre-wrap leading-relaxed">{msg.text}</p>
                    <p className="text-xs opacity-70 mt-2 text-right">
                      {msg.createdAt.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                    </p>
                  </div>
                </div>
              </div>
            ))}

            {/* Loading Indicator */}
            {isLoading && (
              <div className="flex justify-start">
                <div className="flex items-start gap-4 max-w-[80%]">
                  <div className="w-10 h-10 rounded-full bg-gradient-to-r from-cyan-400 to-blue-500 flex items-center justify-center shadow-lg flex-shrink-0">
                    <span className="text-white text-xs font-bold">AI</span>
                  </div>
                  <div className="bg-white/10 text-white rounded-2xl px-5 py-4 rounded-bl-md border border-white/20 backdrop-blur-sm">
                    <div className="flex space-x-2">
                      <div className="w-3 h-3 bg-purple-300 rounded-full animate-bounce"></div>
                      <div className="w-3 h-3 bg-purple-300 rounded-full animate-bounce" style={{ animationDelay: '0.1s' }}></div>
                      <div className="w-3 h-3 bg-purple-300 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }}></div>
                    </div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className="p-6 bg-white/5 border-t border-white/20">
            <form
              onSubmit={(e) => {
                e.preventDefault();
                handleSendMessage();
              }}
              className="flex items-center space-x-4"
            >
              <input
                type="text"
                placeholder={selectedText ? `Ask about: "${selectedText.substring(0, 30)}..."` : 'Ask anything about the book...'}
                value={input}
                onChange={(e) => setInput(e.target.value)}
                className="flex-1 bg-white/10 text-white border border-white/30 rounded-xl px-5 py-4 focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-transparent placeholder:text-gray-400 backdrop-blur-sm text-lg"
                disabled={isLoading}
              />
              <button
                type="submit"
                disabled={isLoading}
                className="p-4 rounded-xl bg-gradient-to-r from-purple-600 to-blue-600 text-white hover:from-purple-700 hover:to-blue-700 transition-all disabled:opacity-50 disabled:cursor-not-allowed shadow-lg hover:shadow-xl"
              >
                <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
                  <path d="M22 2L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </form>
          </div>
        </div>
      </div>
    );
  } else {
    // Floating chat button only
    return (
      <button
        onClick={toggleChat}
        className="fixed bottom-6 right-6 z-[9999] p-5 rounded-full shadow-2xl transition-transform duration-300 bg-gradient-to-br from-purple-600 via-pink-600 to-blue-600 hover:scale-110 text-white font-bold border-4 border-white flex items-center justify-center cursor-pointer"
        style={{ width: '70px', height: '70px', fontSize: '24px' }}
      >
        🤖
      </button>
    );
  }
}
