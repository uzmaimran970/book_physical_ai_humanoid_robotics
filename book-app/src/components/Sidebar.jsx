import React, { useState } from 'react';
import { useNavigate, useParams } from 'react-router-dom';
import { bookData, searchChapters } from '../data/bookContent';
import { Search, ChevronDown, ChevronRight, Home, BookOpen } from 'lucide-react';

const Sidebar = ({ onChapterSelect }) => {
  const navigate = useNavigate();
  const { chapterId } = useParams();
  const [searchQuery, setSearchQuery] = useState('');
  const [expandedModules, setExpandedModules] = useState(
    bookData.modules.map(m => m.id)
  );

  const toggleModule = (moduleId) => {
    setExpandedModules(prev =>
      prev.includes(moduleId)
        ? prev.filter(id => id !== moduleId)
        : [...prev, moduleId]
    );
  };

  const handleChapterClick = (chapterId) => {
    navigate(`/chapter/${chapterId}`);
    if (onChapterSelect) {
      onChapterSelect();
    }
  };

  const goHome = () => {
    navigate('/');
    if (onChapterSelect) {
      onChapterSelect();
    }
  };

  // Filter chapters based on search
  const getFilteredModules = () => {
    if (!searchQuery.trim()) {
      return bookData.modules;
    }

    const searchResults = searchChapters(searchQuery);
    const filteredModules = bookData.modules
      .map(module => ({
        ...module,
        chapters: module.chapters.filter(chapter =>
          searchResults.some(result => result.id === chapter.id)
        )
      }))
      .filter(module => module.chapters.length > 0);

    return filteredModules;
  };

  const filteredModules = getFilteredModules();

  return (
    <div className="h-full flex flex-col bg-white dark:bg-gray-800">
      {/* Header */}
      <div className="p-4 border-b border-gray-200 dark:border-gray-700">
        <button
          onClick={goHome}
          className="w-full flex items-center gap-3 px-4 py-3 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-700 transition-colors group"
        >
          <Home className="w-5 h-5 text-primary-600 dark:text-primary-400" />
          <div className="flex-1 text-left">
            <h2 className="font-bold text-gray-800 dark:text-white text-sm">
              Physical AI & Robotics
            </h2>
            <p className="text-xs text-gray-500 dark:text-gray-400">
              Interactive Textbook
            </p>
          </div>
        </button>
      </div>

      {/* Search Bar */}
      <div className="p-4 border-b border-gray-200 dark:border-gray-700">
        <div className="relative">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-400" />
          <input
            type="text"
            placeholder="Search chapters..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            className="input-field pl-10 text-sm"
          />
        </div>
      </div>

      {/* Navigation */}
      <nav className="flex-1 overflow-y-auto p-4 space-y-2">
        {filteredModules.length === 0 ? (
          <div className="text-center py-8 text-gray-500 dark:text-gray-400">
            <BookOpen className="w-12 h-12 mx-auto mb-2 opacity-50" />
            <p className="text-sm">No chapters found</p>
          </div>
        ) : (
          filteredModules.map((module) => {
            const isExpanded = expandedModules.includes(module.id);

            return (
              <div key={module.id} className="space-y-1">
                {/* Module Header */}
                <button
                  onClick={() => toggleModule(module.id)}
                  className="w-full flex items-center gap-2 px-3 py-2 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-700 transition-colors text-left"
                >
                  <div className="flex-shrink-0 w-6 h-6 rounded bg-gradient-to-br from-primary-500 to-accent-500 flex items-center justify-center text-white text-xs font-bold">
                    {module.number}
                  </div>
                  <span className="flex-1 text-sm font-semibold text-gray-700 dark:text-gray-200">
                    {module.title}
                  </span>
                  {isExpanded ? (
                    <ChevronDown className="w-4 h-4 text-gray-400" />
                  ) : (
                    <ChevronRight className="w-4 h-4 text-gray-400" />
                  )}
                </button>

                {/* Chapters */}
                {isExpanded && (
                  <div className="ml-4 pl-4 border-l-2 border-gray-200 dark:border-gray-700 space-y-1">
                    {module.chapters.map((chapter) => {
                      const isActive = chapter.id === chapterId;

                      return (
                        <button
                          key={chapter.id}
                          onClick={() => handleChapterClick(chapter.id)}
                          className={`
                            w-full text-left px-3 py-2 rounded-lg text-sm transition-all
                            ${
                              isActive
                                ? 'sidebar-item-active'
                                : 'sidebar-item'
                            }
                          `}
                        >
                          <div className="flex items-start gap-2">
                            <span className="flex-shrink-0 text-xs font-medium">
                              {chapter.number}
                            </span>
                            <span className="flex-1">
                              {chapter.title}
                            </span>
                          </div>
                        </button>
                      );
                    })}
                  </div>
                )}
              </div>
            );
          })
        )}
      </nav>

      {/* Footer Stats */}
      <div className="p-4 border-t border-gray-200 dark:border-gray-700">
        <div className="text-xs text-gray-500 dark:text-gray-400 space-y-1">
          <div className="flex justify-between">
            <span>Total Modules:</span>
            <span className="font-medium">{bookData.modules.length}</span>
          </div>
          <div className="flex justify-between">
            <span>Total Chapters:</span>
            <span className="font-medium">
              {bookData.modules.reduce((sum, m) => sum + m.chapters.length, 0)}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Sidebar;
