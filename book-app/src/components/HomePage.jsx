import React from 'react';
import { useNavigate } from 'react-router-dom';
import { bookData } from '../data/bookContent';
import { BookOpen, ArrowRight, Cpu, Brain, Eye, Sparkles } from 'lucide-react';

const HomePage = () => {
  const navigate = useNavigate();

  const startReading = () => {
    // Navigate to first chapter
    const firstChapter = bookData.modules[0].chapters[0];
    navigate(`/chapter/${firstChapter.id}`);
  };

  const goToModule = (moduleId) => {
    const module = bookData.modules.find(m => m.id === moduleId);
    if (module && module.chapters.length > 0) {
      navigate(`/chapter/${module.chapters[0].id}`);
    }
  };

  const moduleIcons = {
    'module-1': Cpu,
    'module-2': Brain,
    'module-3': Sparkles,
    'module-4': Eye,
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 via-white to-purple-50 dark:from-gray-900 dark:via-gray-800 dark:to-gray-900">
      {/* Hero Section */}
      <div className="container mx-auto px-4 py-16">
        <div className="text-center max-w-4xl mx-auto mb-16 animate-fade-in">
          {/* Logo/Icon */}
          <div className="flex justify-center mb-6">
            <div className="p-4 bg-gradient-to-r from-primary-600 to-accent-600 rounded-2xl shadow-2xl">
              <BookOpen className="w-16 h-16 text-white" />
            </div>
          </div>

          {/* Title */}
          <h1 className="text-5xl md:text-6xl font-bold mb-6">
            <span className="gradient-text">
              {bookData.title}
            </span>
          </h1>

          {/* Subtitle */}
          <p className="text-xl md:text-2xl text-gray-600 dark:text-gray-300 mb-4">
            {bookData.subtitle}
          </p>

          {/* Description */}
          <p className="text-lg text-gray-500 dark:text-gray-400 mb-8 max-w-2xl mx-auto">
            {bookData.description}
          </p>

          {/* CTA Button */}
          <button
            onClick={startReading}
            className="btn-primary text-lg px-8 py-4 inline-flex items-center gap-2 group"
          >
            Start Learning
            <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
          </button>
        </div>

        {/* Modules Grid */}
        <div className="max-w-6xl mx-auto">
          <h2 className="text-3xl font-bold text-center mb-12 text-gray-800 dark:text-white">
            Course Modules
          </h2>

          <div className="grid md:grid-cols-2 gap-6">
            {bookData.modules.map((module, index) => {
              const IconComponent = moduleIcons[module.id] || BookOpen;

              return (
                <div
                  key={module.id}
                  className="card p-6 hover:shadow-2xl hover:-translate-y-1 transition-all cursor-pointer animate-fade-in"
                  style={{ animationDelay: `${index * 100}ms` }}
                  onClick={() => goToModule(module.id)}
                >
                  <div className="flex items-start gap-4">
                    {/* Module Number */}
                    <div className="flex-shrink-0">
                      <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-primary-500 to-accent-500 flex items-center justify-center text-white font-bold text-lg shadow-lg">
                        {module.number}
                      </div>
                    </div>

                    {/* Module Content */}
                    <div className="flex-1">
                      <div className="flex items-center gap-2 mb-2">
                        <IconComponent className="w-5 h-5 text-primary-600 dark:text-primary-400" />
                        <h3 className="text-xl font-bold text-gray-800 dark:text-white">
                          {module.title}
                        </h3>
                      </div>

                      <p className="text-gray-600 dark:text-gray-300 mb-4">
                        {module.description}
                      </p>

                      {/* Chapters List */}
                      <div className="space-y-2">
                        {module.chapters.map((chapter) => (
                          <div
                            key={chapter.id}
                            className="flex items-center gap-2 text-sm text-gray-500 dark:text-gray-400 hover:text-primary-600 dark:hover:text-primary-400 transition-colors"
                          >
                            <div className="w-1.5 h-1.5 rounded-full bg-primary-500" />
                            <span>Chapter {chapter.number}: {chapter.title}</span>
                          </div>
                        ))}
                      </div>

                      {/* Start Button */}
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          goToModule(module.id);
                        }}
                        className="mt-4 text-primary-600 dark:text-primary-400 font-medium inline-flex items-center gap-1 hover:gap-2 transition-all"
                      >
                        Start Module
                        <ArrowRight className="w-4 h-4" />
                      </button>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Features Section */}
        <div className="max-w-4xl mx-auto mt-20">
          <div className="grid md:grid-cols-3 gap-6 text-center">
            <div className="p-6">
              <div className="w-12 h-12 mx-auto mb-4 rounded-full bg-primary-100 dark:bg-primary-900/30 flex items-center justify-center">
                <BookOpen className="w-6 h-6 text-primary-600 dark:text-primary-400" />
              </div>
              <h3 className="font-semibold text-gray-800 dark:text-white mb-2">
                Comprehensive Content
              </h3>
              <p className="text-sm text-gray-600 dark:text-gray-400">
                12 in-depth chapters covering all aspects of modern robotics
              </p>
            </div>

            <div className="p-6">
              <div className="w-12 h-12 mx-auto mb-4 rounded-full bg-accent-100 dark:bg-accent-900/30 flex items-center justify-center">
                <Cpu className="w-6 h-6 text-accent-600 dark:text-accent-400" />
              </div>
              <h3 className="font-semibold text-gray-800 dark:text-white mb-2">
                Practical Examples
              </h3>
              <p className="text-sm text-gray-600 dark:text-gray-400">
                Real-world code examples and hands-on exercises
              </p>
            </div>

            <div className="p-6">
              <div className="w-12 h-12 mx-auto mb-4 rounded-full bg-primary-100 dark:bg-primary-900/30 flex items-center justify-center">
                <Brain className="w-6 h-6 text-primary-600 dark:text-primary-400" />
              </div>
              <h3 className="font-semibold text-gray-800 dark:text-white mb-2">
                Cutting-Edge Topics
              </h3>
              <p className="text-sm text-gray-600 dark:text-gray-400">
                From ROS 2 basics to Vision-Language-Action models
              </p>
            </div>
          </div>
        </div>
      </div>

      {/* Footer */}
      <footer className="border-t border-gray-200 dark:border-gray-700 py-8 mt-16">
        <div className="container mx-auto px-4 text-center text-gray-600 dark:text-gray-400">
          <p>© 2025 Physical AI & Humanoid Robotics. All rights reserved.</p>
        </div>
      </footer>
    </div>
  );
};

export default HomePage;
