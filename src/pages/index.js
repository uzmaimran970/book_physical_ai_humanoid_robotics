import React from 'react';
import { motion } from 'framer-motion';
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
    bg: 'bg-book-yellow-50 dark:bg-book-yellow-900/10',
    border: 'border-book-yellow-200 dark:border-book-yellow-800',
    text: 'text-book-yellow-700 dark:text-book-yellow-400',
    iconBg: 'bg-book-yellow-100 dark:bg-book-yellow-900/30',
    hover: 'hover:border-book-yellow-400 dark:hover:border-book-yellow-600 hover:shadow-md'
  },
  indigo: {
    bg: 'bg-book-pink-50 dark:bg-book-pink-900/10',
    border: 'border-book-pink-200 dark:border-book-pink-800',
    text: 'text-book-pink-600 dark:text-book-pink-400',
    iconBg: 'bg-book-pink-100 dark:bg-book-pink-900/30',
    hover: 'hover:border-book-pink-400 dark:hover:border-book-pink-600 hover:shadow-md'
  },
  purple: {
    bg: 'bg-gradient-to-br from-book-yellow-50 to-book-pink-50 dark:from-book-yellow-900/10 dark:to-book-pink-900/10',
    border: 'border-book-yellow-200 dark:border-book-yellow-800',
    text: 'text-book-yellow-700 dark:text-book-yellow-400',
    iconBg: 'bg-gradient-to-br from-book-yellow-100 to-book-pink-100 dark:from-book-yellow-900/30 dark:to-book-pink-900/30',
    hover: 'hover:border-book-pink-300 dark:hover:border-book-pink-700 hover:shadow-md'
  },
  pink: {
    bg: 'bg-book-pink-50 dark:bg-book-pink-900/10',
    border: 'border-book-pink-200 dark:border-book-pink-800',
    text: 'text-book-pink-600 dark:text-book-pink-400',
    iconBg: 'bg-book-pink-100 dark:bg-book-pink-900/30',
    hover: 'hover:border-book-pink-400 dark:hover:border-book-pink-600 hover:shadow-md'
  }
};

// Animation variants for professional, subtle animations
const fadeUpVariants = {
  hidden: { opacity: 0, y: 30 },
  visible: {
    opacity: 1,
    y: 0,
    transition: { duration: 0.8, ease: [0.25, 0.1, 0.25, 1] }
  }
};

const fadeInVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: { duration: 0.6, ease: 'easeOut' }
  }
};

const staggerContainer = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.15,
      delayChildren: 0.2
    }
  }
};

const slideInVariants = {
  hidden: { opacity: 0, x: -30 },
  visible: {
    opacity: 1,
    x: 0,
    transition: { duration: 0.7, ease: [0.25, 0.1, 0.25, 1] }
  }
};

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A Comprehensive Guide to Modern Robotics and AI">

      {/* Hero Section */}
      <section className="relative overflow-hidden bg-gradient-to-br from-book-yellow-50 via-white to-book-pink-50 dark:from-gray-900 dark:via-gray-800 dark:to-gray-900 py-16 md:py-24 lg:py-32">
        {/* Animated background gradient */}
        <div className="absolute inset-0 bg-gradient-to-br from-book-yellow-400/10 via-transparent to-book-pink-500/10 dark:from-book-yellow-500/5 dark:to-book-pink-500/5 animate-pulse-slow"></div>

        <div className="container relative mx-auto px-4 max-w-6xl">
          <motion.div
            className="text-center mb-14"
            initial="hidden"
            animate="visible"
            variants={staggerContainer}
          >
            {/* Book badge */}
            <motion.div
              className="inline-flex items-center gap-2 px-4 py-2 mb-6 bg-book-yellow-100 dark:bg-book-yellow-900/20 border border-book-yellow-300 dark:border-book-yellow-700 rounded-full"
              variants={fadeInVariants}
            >
              <svg className="w-4 h-4 text-book-yellow-600 dark:text-book-yellow-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
              </svg>
              <span className="text-sm font-semibold text-book-yellow-700 dark:text-book-yellow-300">AI-Native Interactive Textbook</span>
            </motion.div>

            <motion.h1
              className="text-4xl md:text-6xl lg:text-7xl font-extrabold mb-6 bg-gradient-to-r from-gray-900 via-book-yellow-700 to-book-pink-600 dark:from-white dark:via-book-yellow-400 dark:to-book-pink-400 bg-clip-text text-transparent leading-tight"
              variants={fadeUpVariants}
            >
              Physical AI &<br className="sm:hidden" /> Humanoid Robotics
            </motion.h1>

            <motion.p
              className="text-lg md:text-xl lg:text-2xl text-gray-700 dark:text-gray-300 max-w-3xl mx-auto font-medium leading-relaxed"
              variants={fadeUpVariants}
            >
              Master intelligent robotics with <span className="text-book-yellow-600 dark:text-book-yellow-400 font-semibold">ROS 2</span>, <span className="text-book-pink-600 dark:text-book-pink-400 font-semibold">NVIDIA Isaac</span>, and cutting-edge <span className="text-book-yellow-600 dark:text-book-yellow-400 font-semibold">Vision-Language-Action</span> models
            </motion.p>
          </motion.div>

          {/* Quick Stats */}
          <motion.div
            className="grid grid-cols-3 gap-4 md:gap-8 max-w-3xl mx-auto mb-14"
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true, margin: "-100px" }}
            variants={staggerContainer}
          >
            <motion.div
              className="text-center p-4 bg-white/60 dark:bg-gray-800/60 backdrop-blur-sm rounded-xl border border-book-yellow-200 dark:border-book-yellow-800/50 transition-all duration-300 hover:shadow-lg hover:-translate-y-1"
              variants={fadeUpVariants}
            >
              <div className="text-3xl md:text-5xl font-bold bg-gradient-to-br from-book-yellow-500 to-book-yellow-600 bg-clip-text text-transparent">4</div>
              <div className="text-sm md:text-base text-gray-700 dark:text-gray-400 mt-2 font-medium">Modules</div>
            </motion.div>
            <motion.div
              className="text-center p-4 bg-white/60 dark:bg-gray-800/60 backdrop-blur-sm rounded-xl border border-book-pink-200 dark:border-book-pink-800/50 transition-all duration-300 hover:shadow-lg hover:-translate-y-1"
              variants={fadeUpVariants}
            >
              <div className="text-3xl md:text-5xl font-bold bg-gradient-to-br from-book-pink-500 to-book-pink-600 bg-clip-text text-transparent">12</div>
              <div className="text-sm md:text-base text-gray-700 dark:text-gray-400 mt-2 font-medium">Chapters</div>
            </motion.div>
            <motion.div
              className="text-center p-4 bg-white/60 dark:bg-gray-800/60 backdrop-blur-sm rounded-xl border border-book-yellow-200 dark:border-book-yellow-800/50 transition-all duration-300 hover:shadow-lg hover:-translate-y-1"
              variants={fadeUpVariants}
            >
              <div className="text-3xl md:text-5xl font-bold bg-gradient-to-br from-book-yellow-500 to-book-pink-600 bg-clip-text text-transparent">100+</div>
              <div className="text-sm md:text-base text-gray-700 dark:text-gray-400 mt-2 font-medium">Examples</div>
            </motion.div>
          </motion.div>

          {/* CTA Buttons */}
          <motion.div
            className="flex flex-col sm:flex-row gap-4 justify-center items-center"
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true }}
            variants={fadeUpVariants}
          >
            <Link
              to="/docs/intro"
              className="group inline-flex items-center justify-center px-8 py-4 text-base md:text-lg font-semibold rounded-xl text-white bg-gradient-to-r from-book-pink-500 to-book-pink-600 hover:from-book-pink-600 hover:to-book-pink-700 shadow-lg transition-all duration-300 hover:shadow-[0_8px_30px_rgba(236,72,153,0.4)] hover:scale-105">
              Start Reading
              <svg className="ml-2 w-5 h-5 group-hover:translate-x-1 transition-transform duration-300" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 7l5 5m0 0l-5 5m5-5H6" />
              </svg>
            </Link>
            <Link
              to="/docs/intro"
              className="group inline-flex items-center justify-center px-8 py-4 text-base md:text-lg font-semibold rounded-xl text-gray-900 dark:text-white bg-white dark:bg-gray-800 border-2 border-book-yellow-400 dark:border-book-yellow-600 hover:bg-book-yellow-50 dark:hover:bg-gray-700 shadow-md transition-all duration-300 hover:shadow-lg hover:scale-105">
              Browse Modules
              <svg className="ml-2 w-5 h-5 transition-transform duration-300" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
              </svg>
            </Link>
          </motion.div>
        </div>
      </section>

      {/* Course Modules */}
      <section className="py-16 md:py-24 bg-gradient-to-b from-white to-gray-50 dark:from-gray-800 dark:to-gray-900">
        <div className="container mx-auto px-4 max-w-6xl">
          <motion.div
            className="text-center mb-16"
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true, margin: "-100px" }}
            variants={slideInVariants}
          >
            <h2 className="text-3xl md:text-4xl lg:text-5xl font-bold bg-gradient-to-r from-gray-900 to-book-yellow-700 dark:from-white dark:to-book-yellow-400 bg-clip-text text-transparent mb-4">
              Course Modules
            </h2>
            <p className="text-lg md:text-xl text-gray-600 dark:text-gray-400 max-w-2xl mx-auto">
              Structured learning path from fundamentals to advanced topics
            </p>
          </motion.div>

          <motion.div
            className="grid md:grid-cols-2 gap-6 lg:gap-8"
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true, margin: "-100px" }}
            variants={staggerContainer}
          >
            {modules.map((module, idx) => {
              const colors = colorMap[module.color];
              return (
                <motion.div
                  key={module.id}
                  className={`group border-2 ${colors.border} ${colors.bg} rounded-2xl p-6 md:p-8 transition-all duration-300 ${colors.hover} cursor-pointer`}
                  variants={fadeUpVariants}
                  whileHover={{
                    y: -8,
                    scale: 1.02,
                    transition: { duration: 0.3, ease: "easeOut" }
                  }}
                >
                  <div className="flex items-start gap-5">
                    <div className={`${colors.iconBg} ${colors.text} p-4 rounded-xl flex-shrink-0 group-hover:scale-110 transition-transform duration-300`}>
                      {module.icon}
                    </div>
                    <div className="flex-1 min-w-0">
                      <div className="flex items-baseline gap-2 mb-3">
                        <span className={`${colors.text} text-sm font-bold uppercase tracking-wide`}>
                          Module {module.id}
                        </span>
                      </div>
                      <h3 className="text-xl md:text-2xl font-bold text-gray-900 dark:text-white mb-4 leading-tight">
                        {module.title}
                      </h3>
                      <ul className="space-y-3">
                        {module.chapters.map((chapter, idx) => (
                          <li key={idx} className="flex items-start gap-3 text-sm md:text-base text-gray-700 dark:text-gray-300">
                            <svg className="w-5 h-5 mt-0.5 flex-shrink-0 text-book-yellow-500 dark:text-book-yellow-400" fill="currentColor" viewBox="0 0 20 20">
                              <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                            </svg>
                            <span className="flex-1 font-medium">{chapter}</span>
                          </li>
                        ))}
                      </ul>
                    </div>
                  </div>
                </motion.div>
              );
            })}
          </motion.div>

          <div className="mt-14 text-center">
            <Link
              to="/docs/intro"
              className={`inline-flex items-center gap-2 text-lg font-semibold text-book-pink-600 dark:text-book-pink-400 hover:text-book-pink-700 dark:hover:text-book-pink-300 transition-colors group`}>
              View full curriculum
              <svg className="w-5 h-5 group-hover:translate-x-1 transition-transform" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
              </svg>
            </Link>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="py-16 md:py-24 bg-gradient-to-b from-gray-50 to-white dark:from-gray-900 dark:to-gray-800">
        <div className="container mx-auto px-4 max-w-6xl">
          <div className="grid md:grid-cols-3 gap-8 lg:gap-12">
            <div className="text-center group">
              <div className="inline-flex items-center justify-center w-16 h-16 mb-6 rounded-2xl bg-gradient-to-br from-book-yellow-100 to-book-yellow-200 dark:from-book-yellow-900/30 dark:to-book-yellow-800/30 text-book-yellow-600 dark:text-book-yellow-400 group-hover:scale-110 transition-transform duration-300 shadow-lg">
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
                </svg>
              </div>
              <h3 className="text-xl md:text-2xl font-bold text-gray-900 dark:text-white mb-3">
                Comprehensive Content
              </h3>
              <p className="text-base md:text-lg text-gray-600 dark:text-gray-400 leading-relaxed">
                From ROS 2 basics to advanced VLA models
              </p>
            </div>

            <div className="text-center group">
              <div className="inline-flex items-center justify-center w-16 h-16 mb-6 rounded-2xl bg-gradient-to-br from-book-pink-100 to-book-pink-200 dark:from-book-pink-900/30 dark:to-book-pink-800/30 text-book-pink-600 dark:text-book-pink-400 group-hover:scale-110 transition-transform duration-300 shadow-lg">
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 20l4-16m4 4l4 4-4 4M6 16l-4-4 4-4" />
                </svg>
              </div>
              <h3 className="text-xl md:text-2xl font-bold text-gray-900 dark:text-white mb-3">
                Practical Examples
              </h3>
              <p className="text-base md:text-lg text-gray-600 dark:text-gray-400 leading-relaxed">
                100+ code examples and hands-on exercises
              </p>
            </div>

            <div className="text-center group">
              <div className="inline-flex items-center justify-center w-16 h-16 mb-6 rounded-2xl bg-gradient-to-br from-book-yellow-100 via-book-pink-100 to-book-pink-200 dark:from-book-yellow-900/30 dark:via-book-pink-900/30 dark:to-book-pink-800/30 text-book-pink-600 dark:text-book-pink-400 group-hover:scale-110 transition-transform duration-300 shadow-lg">
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              <h3 className="text-xl md:text-2xl font-bold text-gray-900 dark:text-white mb-3">
                Modern Technologies
              </h3>
              <p className="text-base md:text-lg text-gray-600 dark:text-gray-400 leading-relaxed">
                NVIDIA Isaac, Gazebo, Unity, and more
              </p>
            </div>
          </div>
        </div>
      </section>
    </Layout>
  );
}
