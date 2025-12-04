// 🚀 Out-of-the-box NEXT-LEVEL UI for Book RAG Chatbot
// Theme: Physical AI & Humanoid Robotics
// Features: 3D metallic UI, robot avatar, page-flip animation, mobile optimized, premium design

import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface ChatMessage {
  text: string;
  sender: 'user' | 'bot';
  timestamp: string;
}

export default function ChatbotWidget() {
  const { siteConfig: { customFields } } = useDocusaurusContext();

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const API_BASE_URL = customFields.API_BASE_URL as string;

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection()?.toString().trim();
      if (selection) {
        setSelectedText(selection);
        setIsOpen(true);
      }
    };
    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, []);

  const handleSendMessage = async (e?: any) => {
    e?.preventDefault();
    if (!input.trim() && !selectedText.trim()) return;

    const userMessage: ChatMessage = {
      text: input || `Question about: "${selectedText}"`,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    const endpoint = selectedText ? `${API_BASE_URL}/query_selected_text` : `${API_BASE_URL}/chat`;
    const body = selectedText
      ? JSON.stringify({ selected_text: selectedText, user_question: input })
      : JSON.stringify({ query: input });

    try {
      const response = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body,
      });

      const data = await response.json();

      setMessages(prev => [
        ...prev,
        { text: data.response, sender: 'bot', timestamp: new Date().toLocaleTimeString() },
      ]);
    } catch {
      setMessages(prev => [
        ...prev,
        { text: 'Connection error. Try again later.', sender: 'bot', timestamp: new Date().toLocaleTimeString() },
      ]);
    } finally {
      setIsLoading(false);
      setSelectedText('');
    }
  };

  const toggleChatbot = () => setIsOpen(!isOpen);

  return (
    <>
      {/* Floating Robotics Button */}
      <button
        onClick={toggleChatbot}
        className={clsx(
          'fixed bottom-6 right-6 z-[9999] p-4 rounded-full shadow-xl transition-all duration-500',
          'bg-gradient-to-br from-gray-900 to-gray-700 hover:scale-110 hover:shadow-2xl',
          'border border-gray-500/50 backdrop-blur-xl',
          'text-cyan-300 font-semibold tracking-widest',
          isOpen ? 'rotate-45' : 'rotate-0'
        )}
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chat Window */}
      <div
        className={clsx(
          'fixed bottom-24 right-6 z-[9999] w-[380px] h-[560px] rounded-xl overflow-hidden transition-all duration-500',
          'bg-gradient-to-br from-gray-950 via-gray-900 to-gray-800',
          'border border-gray-700/60 shadow-2xl backdrop-blur-2xl',
          'transform-gpu',
          isOpen ? 'opacity-100 translate-y-0 scale-100' : 'opacity-0 translate-y-10 scale-95 pointer-events-none'
        )}
      >
        {/* Header */}
        <div className="p-4 bg-gradient-to-r from-purple-700 to-blue-600 flex items-center justify-between shadow-lg">
          <div>
            <h3 className="text-white font-bold text-lg">Physical AI & Robotics</h3>
            <p className="text-purple-200 text-xs">Your Book AI Tutor</p>
          </div>
          <span className="text-white text-xl cursor-pointer" onClick={toggleChatbot}>✕</span>
        </div>

        {/* Messages */}
        <div className="flex-1 p-4 overflow-y-auto space-y-4 custom-scrollbar">
          {messages.map((msg, i) => (
            <div key={i} className={clsx('flex items-end', msg.sender === 'user' ? 'justify-end' : 'justify-start')}>

              {/* Robotic Avatar */}
              {msg.sender === 'bot' && (
                <img
                  src="https://cdn-icons-png.flaticon.com/512/4712/4712100.png"
                  className="w-10 h-10 rounded-full mr-3 shadow-md"
                />
              )}

              {/* Message Bubble */}
              <div
                className={clsx(
                  'max-w-[72%] p-3 rounded-2xl text-sm shadow-xl backdrop-blur-md',
                  msg.sender === 'user'
                    ? 'bg-gradient-to-br from-blue-600 to-purple-700 text-white rounded-br-none'
                    : 'bg-gray-200/20 text-gray-100 border border-gray-500/30 rounded-bl-none'
                )}
              >
                {msg.text}
                <span className="block text-[10px] opacity-70 mt-1">{msg.timestamp}</span>
              </div>
            </div>
          ))}

          {/* Typing Indicator */}
          {isLoading && (
            <div className="flex items-center space-x-2 text-gray-200">
              <img
                src="https://cdn-icons-png.flaticon.com/512/4712/4712100.png"
                className="w-8 h-8 rounded-full"
              />
              <div className="flex space-x-1 p-3 rounded-xl bg-gray-700/50 border border-gray-600/40 animate-pulse">
                <span className="w-2 h-2 rounded-full bg-gray-300 animate-bounce"></span>
                <span className="w-2 h-2 rounded-full bg-gray-300 animate-bounce delay-150"></span>
                <span className="w-2 h-2 rounded-full bg-gray-300 animate-bounce delay-300"></span>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Input */}
        <form onSubmit={handleSendMessage} className="p-4 flex items-center space-x-2 bg-gray-900/60 border-t border-gray-700/50">
          <input
            value={input}
            onChange={(e) => setInput(e.target.value)}
            placeholder={selectedText ? `Ask about "${selectedText}"...` : 'Ask anything from the book...'}
            className="flex-1 p-3 rounded-xl bg-gray-800 text-gray-200 border border-gray-700 focus:ring-2 focus:ring-cyan-400 outline-none"
          />

          <button
            type="submit"
            className="p-3 px-4 rounded-xl bg-gradient-to-br from-cyan-600 to-blue-700 text-white font-bold hover:scale-105 transition-all shadow-lg"
          >
            ➤
          </button>
        </form>
      </div>
    </>
  );
}
