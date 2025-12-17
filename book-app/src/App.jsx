import React, { useState, useEffect } from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import HomePage from './components/HomePage';
import ChapterReader from './components/ChapterReader';
import Sidebar from './components/Sidebar';
import ThemeToggle from './components/ThemeToggle';
import ChatBot from './components/ChatBot';
import { Menu, X } from 'lucide-react';

function App() {
  const [isDarkMode, setIsDarkMode] = useState(() => {
    const saved = localStorage.getItem('darkMode');
    return saved ? JSON.parse(saved) : false;
  });

  const [isSidebarOpen, setIsSidebarOpen] = useState(false);

  // Apply dark mode class to document
  useEffect(() => {
    if (isDarkMode) {
      document.documentElement.classList.add('dark');
    } else {
      document.documentElement.classList.remove('dark');
    }
    localStorage.setItem('darkMode', JSON.stringify(isDarkMode));
  }, [isDarkMode]);

  const toggleTheme = () => {
    setIsDarkMode(!isDarkMode);
  };

  const toggleSidebar = () => {
    setIsSidebarOpen(!isSidebarOpen);
  };

  const closeSidebar = () => {
    setIsSidebarOpen(false);
  };

  return (
    <Router>
      <div className="min-h-screen bg-gray-50 dark:bg-gray-900 transition-colors duration-200">
        <ChatBot />
        <Routes>
          <Route path="/" element={<HomePage />} />
          <Route
            path="/chapter/:chapterId"
            element={
              <div className="flex h-screen overflow-hidden">
                {/* Mobile Sidebar Overlay */}
                {isSidebarOpen && (
                  <div
                    className="fixed inset-0 bg-black bg-opacity-50 z-40 lg:hidden"
                    onClick={closeSidebar}
                  />
                )}

                {/* Sidebar */}
                <aside
                  className={`
                    fixed lg:static inset-y-0 left-0 z-50
                    w-80 bg-white dark:bg-gray-800
                    border-r border-gray-200 dark:border-gray-700
                    transform transition-transform duration-300 ease-in-out
                    ${isSidebarOpen ? 'translate-x-0' : '-translate-x-full lg:translate-x-0'}
                  `}
                >
                  <Sidebar onChapterSelect={closeSidebar} />
                </aside>

                {/* Main Content */}
                <div className="flex-1 flex flex-col overflow-hidden">
                  {/* Top Navigation Bar */}
                  <header className="bg-white dark:bg-gray-800 border-b border-gray-200 dark:border-gray-700 px-4 py-3 flex items-center justify-between">
                    <button
                      onClick={toggleSidebar}
                      className="lg:hidden p-2 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-700 transition-colors"
                      aria-label="Toggle sidebar"
                    >
                      {isSidebarOpen ? (
                        <X className="w-6 h-6 text-gray-700 dark:text-gray-300" />
                      ) : (
                        <Menu className="w-6 h-6 text-gray-700 dark:text-gray-300" />
                      )}
                    </button>

                    <h1 className="text-lg font-semibold text-gray-800 dark:text-white hidden sm:block">
                      Physical AI & Humanoid Robotics
                    </h1>

                    <ThemeToggle isDarkMode={isDarkMode} toggleTheme={toggleTheme} />
                  </header>

                  {/* Chapter Content */}
                  <main className="flex-1 overflow-auto">
                    <ChapterReader />
                  </main>
                </div>
              </div>
            }
          />
          <Route path="*" element={<Navigate to="/" replace />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
