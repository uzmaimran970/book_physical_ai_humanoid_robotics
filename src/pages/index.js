import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

const modules = [
  {
    id: 1,
    title: 'The Robotic Nervous System (ROS 2)',
    chapters: [
      'ROS 2 Nodes and Topics',
      'ROS 2 Services and Actions',
      'ROS 2 Launch Files and Parameters'
    ],
    icon: (
      <svg className="w-5 h-5 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
      </svg>
    ),
    color: 'blue'
  },
  {
    id: 2,
    title: 'The Digital Twin (Gazebo & Unity)',
    chapters: [
      'Introduction to Digital Twins',
      'Gazebo Simulation',
      'Unity Simulation and Robotics'
    ],
    icon: (
      <svg className="w-5 h-5 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 21a4 4 0 01-4-4V5a2 2 0 012-2h4a2 2 0 012 2v12a4 4 0 01-4 4zm0 0h12a2 2 0 002-2v-4a2 2 0 00-2-2h-2.343M11 7.343l1.657-1.657a2 2 0 012.828 0l2.829 2.829a2 2 0 010 2.828l-8.486 8.485M7 17h.01" />
      </svg>
    ),
    color: 'indigo'
  },
  {
    id: 3,
    title: 'The AI-Robot Brain (NVIDIA Isaac™)',
    chapters: [
      'Introduction to NVIDIA Isaac',
      'Isaac Navigation Stack',
      'Isaac Perception and Manipulation'
    ],
    icon: (
      <svg className="w-5 h-5 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
      </svg>
    ),
    color: 'purple'
  },
  {
    id: 4,
    title: 'Vision-Language-Action (VLA)',
    chapters: [
      'Vision-Language Models for Robotics',
      'Language-Driven Robot Control',
      'End-to-End VLA Systems'
    ],
    icon: (
      <svg className="w-5 h-5 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
      </svg>
    ),
    color: 'pink'
  }
];

const colorMap = {
  blue: {
    bg: 'bg-blue-50 dark:bg-blue-900/10',
    border: 'border-blue-200 dark:border-blue-800',
    text: 'text-blue-600 dark:text-blue-400',
    iconBg: 'bg-blue-100 dark:bg-blue-900/30',
    hover: 'hover:border-blue-300 dark:hover:border-blue-700'
  },
  indigo: {
    bg: 'bg-indigo-50 dark:bg-indigo-900/10',
    border: 'border-indigo-200 dark:border-indigo-800',
    text: 'text-indigo-600 dark:text-indigo-400',
    iconBg: 'bg-indigo-100 dark:bg-indigo-900/30',
    hover: 'hover:border-indigo-300 dark:hover:border-indigo-700'
  },
  purple: {
    bg: 'bg-purple-50 dark:bg-purple-900/10',
    border: 'border-purple-200 dark:border-purple-800',
    text: 'text-purple-600 dark:text-purple-400',
    iconBg: 'bg-purple-100 dark:bg-purple-900/30',
    hover: 'hover:border-purple-300 dark:hover:border-purple-700'
  },
  pink: {
    bg: 'bg-pink-50 dark:bg-pink-900/10',
    border: 'border-pink-200 dark:border-pink-800',
    text: 'text-pink-600 dark:text-pink-400',
    iconBg: 'bg-pink-100 dark:bg-pink-900/30',
    hover: 'hover:border-pink-300 dark:hover:border-pink-700'
  }
};

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A Comprehensive Guide to Modern Robotics and AI">

      {/* Hero Section */}
      <section className="bg-gradient-to-b from-blue-50 to-white dark:from-gray-900 dark:to-gray-800 py-12 md:py-20">
        <div className="container mx-auto px-4 max-w-6xl">
          <div className="text-center mb-12">
            <h1 className="text-3xl md:text-5xl lg:text-6xl font-bold text-gray-900 dark:text-white mb-4">
              Physical AI & Humanoid Robotics
            </h1>
            <p className="text-lg md:text-xl text-gray-600 dark:text-gray-300 max-w-3xl mx-auto">
              A comprehensive guide to building intelligent robots with ROS 2, simulation, NVIDIA Isaac, and Vision-Language-Action models
            </p>
          </div>

          {/* Quick Stats */}
          <div className="grid grid-cols-3 gap-4 md:gap-8 max-w-2xl mx-auto mb-12">
            <div className="text-center">
              <div className="text-2xl md:text-4xl font-bold text-blue-600 dark:text-blue-400">4</div>
              <div className="text-xs md:text-sm text-gray-600 dark:text-gray-400 mt-1">Modules</div>
            </div>
            <div className="text-center">
              <div className="text-2xl md:text-4xl font-bold text-indigo-600 dark:text-indigo-400">12</div>
              <div className="text-xs md:text-sm text-gray-600 dark:text-gray-400 mt-1">Chapters</div>
            </div>
            <div className="text-center">
              <div className="text-2xl md:text-4xl font-bold text-purple-600 dark:text-purple-400">100+</div>
              <div className="text-xs md:text-sm text-gray-600 dark:text-gray-400 mt-1">Examples</div>
            </div>
          </div>

          {/* CTA Buttons */}
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link
              to="/docs/intro"
              className="inline-flex items-center justify-center px-6 py-3 border border-transparent text-base font-medium rounded-lg text-white bg-blue-600 hover:bg-blue-700 dark:bg-blue-500 dark:hover:bg-blue-600 transition-colors">
              Get Started
              <svg className="ml-2 w-4 h-4 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 7l5 5m0 0l-5 5m5-5H6" />
              </svg>
            </Link>
            <Link
              to="/docs/intro"
              className="inline-flex items-center justify-center px-6 py-3 border border-gray-300 dark:border-gray-600 text-base font-medium rounded-lg text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-800 hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors">
              Browse Chapters
            </Link>
          </div>
        </div>
      </section>

      {/* Course Modules */}
      <section className="py-12 md:py-20 bg-white dark:bg-gray-800">
        <div className="container mx-auto px-4 max-w-6xl">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold text-gray-900 dark:text-white mb-3">
              Course Modules
            </h2>
            <p className="text-gray-600 dark:text-gray-400">
              Structured learning path from fundamentals to advanced topics
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            {modules.map((module) => {
              const colors = colorMap[module.color];
              return (
                <div
                  key={module.id}
                  className={`border ${colors.border} ${colors.bg} rounded-lg p-6 transition-all ${colors.hover}`}>
                  <div className="flex items-start gap-4">
                    <div className={`${colors.iconBg} ${colors.text} p-3 rounded-lg flex-shrink-0`}>
                      {module.icon}
                    </div>
                    <div className="flex-1 min-w-0">
                      <div className="flex items-baseline gap-2 mb-2">
                        <span className={`${colors.text} text-sm font-semibold`}>
                          Module {module.id}
                        </span>
                      </div>
                      <h3 className="text-lg font-bold text-gray-900 dark:text-white mb-3">
                        {module.title}
                      </h3>
                      <ul className="space-y-2">
                        {module.chapters.map((chapter, idx) => (
                          <li key={idx} className="flex items-start gap-2 text-sm text-gray-600 dark:text-gray-400">
                            <svg className="w-4 h-4 mt-0.5 flex-shrink-0 text-gray-400 dark:text-gray-500" fill="currentColor" viewBox="0 0 20 20">
                              <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                            </svg>
                            <span className="flex-1">{chapter}</span>
                          </li>
                        ))}
                      </ul>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>

          <div className="mt-12 text-center">
            <Link
              to="/docs/intro"
              className={`inline-flex items-center text-blue-600 dark:text-blue-400 font-medium hover:underline`}>
              View full curriculum
              <svg className="ml-1 w-4 h-4 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
              </svg>
            </Link>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="py-12 md:py-20 bg-gray-50 dark:bg-gray-900">
        <div className="container mx-auto px-4 max-w-6xl">
          <div className="grid md:grid-cols-3 gap-8">
            <div className="text-center">
              <div className="inline-flex items-center justify-center w-12 h-12 mb-4 rounded-lg bg-blue-100 dark:bg-blue-900/30 text-blue-600 dark:text-blue-400">
                <svg className="w-6 h-6 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
                </svg>
              </div>
              <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-2">
                Comprehensive Content
              </h3>
              <p className="text-gray-600 dark:text-gray-400 text-sm">
                From ROS 2 basics to advanced VLA models
              </p>
            </div>

            <div className="text-center">
              <div className="inline-flex items-center justify-center w-12 h-12 mb-4 rounded-lg bg-indigo-100 dark:bg-indigo-900/30 text-indigo-600 dark:text-indigo-400">
                <svg className="w-6 h-6 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 20l4-16m4 4l4 4-4 4M6 16l-4-4 4-4" />
                </svg>
              </div>
              <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-2">
                Practical Examples
              </h3>
              <p className="text-gray-600 dark:text-gray-400 text-sm">
                100+ code examples and hands-on exercises
              </p>
            </div>

            <div className="text-center">
              <div className="inline-flex items-center justify-center w-12 h-12 mb-4 rounded-lg bg-purple-100 dark:bg-purple-900/30 text-purple-600 dark:text-purple-400">
                <svg className="w-6 h-6 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-2">
                Modern Technologies
              </h3>
              <p className="text-gray-600 dark:text-gray-400 text-sm">
                NVIDIA Isaac, Gazebo, Unity, and more
              </p>
            </div>
          </div>
        </div>
      </section>
    </Layout>
  );
}
