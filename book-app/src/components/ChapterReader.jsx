import React, { useEffect, useRef } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import { getChapterById, getModuleById, getNextChapter, getPreviousChapter } from '../data/bookContent';
import { ChevronLeft, ChevronRight, Home } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { oneDark } from 'react-syntax-highlighter/dist/esm/styles/prism';

const ChapterReader = () => {
  const { chapterId } = useParams();
  const navigate = useNavigate();
  const contentRef = useRef(null);

  const chapter = getChapterById(chapterId);
  const module = chapter ? getModuleById(chapter.moduleId) : null;
  const nextChapter = getNextChapter(chapterId);
  const prevChapter = getPreviousChapter(chapterId);

  // Scroll to top when chapter changes
  useEffect(() => {
    if (contentRef.current) {
      contentRef.current.scrollTop = 0;
    }
  }, [chapterId]);

  if (!chapter || !module) {
    return (
      <div className="flex items-center justify-center h-full">
        <div className="text-center">
          <h2 className="text-2xl font-bold text-gray-800 dark:text-white mb-4">
            Chapter Not Found
          </h2>
          <button
            onClick={() => navigate('/')}
            className="btn-primary inline-flex items-center gap-2"
          >
            <Home className="w-4 h-4" />
            Go Home
          </button>
        </div>
      </div>
    );
  }

  return (
    <div ref={contentRef} className="h-full overflow-auto">
      <div className="max-w-4xl mx-auto px-4 py-8">
        {/* Breadcrumb */}
        <div className="mb-6 flex items-center gap-2 text-sm text-gray-600 dark:text-gray-400">
          <button
            onClick={() => navigate('/')}
            className="hover:text-primary-600 dark:hover:text-primary-400 transition-colors"
          >
            Home
          </button>
          <span>/</span>
          <span>Module {module.number}</span>
          <span>/</span>
          <span className="text-gray-800 dark:text-white font-medium">
            Chapter {chapter.number}
          </span>
        </div>

        {/* Module Badge */}
        <div className="inline-flex items-center gap-2 px-3 py-1.5 rounded-full bg-gradient-to-r from-primary-100 to-accent-100 dark:from-primary-900/30 dark:to-accent-900/30 mb-4">
          <div className="w-6 h-6 rounded-full bg-gradient-to-br from-primary-500 to-accent-500 flex items-center justify-center text-white text-xs font-bold">
            {module.number}
          </div>
          <span className="text-sm font-medium text-gray-700 dark:text-gray-300">
            {module.title}
          </span>
        </div>

        {/* Chapter Title */}
        <h1 className="text-4xl md:text-5xl font-bold text-gray-900 dark:text-white mb-6">
          {chapter.title}
        </h1>

        {/* Content */}
        <article className="prose prose-lg dark:prose-invert max-w-none">
          <ReactMarkdown
            components={{
              code({ node, inline, className, children, ...props }) {
                const match = /language-(\w+)/.exec(className || '');
                return !inline && match ? (
                  <SyntaxHighlighter
                    style={oneDark}
                    language={match[1]}
                    PreTag="div"
                    {...props}
                  >
                    {String(children).replace(/\n$/, '')}
                  </SyntaxHighlighter>
                ) : (
                  <code className={className} {...props}>
                    {children}
                  </code>
                );
              },
              h1: ({ children }) => (
                <h1 className="text-3xl font-bold mt-8 mb-4 text-gray-900 dark:text-white">
                  {children}
                </h1>
              ),
              h2: ({ children }) => (
                <h2 className="text-2xl font-bold mt-6 mb-3 text-gray-900 dark:text-white">
                  {children}
                </h2>
              ),
              h3: ({ children }) => (
                <h3 className="text-xl font-semibold mt-4 mb-2 text-gray-800 dark:text-gray-100">
                  {children}
                </h3>
              ),
              p: ({ children }) => (
                <p className="mb-4 text-gray-700 dark:text-gray-300 leading-relaxed">
                  {children}
                </p>
              ),
              ul: ({ children }) => (
                <ul className="list-disc list-inside mb-4 space-y-2 text-gray-700 dark:text-gray-300">
                  {children}
                </ul>
              ),
              ol: ({ children }) => (
                <ol className="list-decimal list-inside mb-4 space-y-2 text-gray-700 dark:text-gray-300">
                  {children}
                </ol>
              ),
              blockquote: ({ children }) => (
                <blockquote className="border-l-4 border-primary-500 pl-4 italic my-4 text-gray-600 dark:text-gray-400">
                  {children}
                </blockquote>
              ),
              table: ({ children }) => (
                <div className="overflow-x-auto my-6">
                  <table className="min-w-full divide-y divide-gray-300 dark:divide-gray-700">
                    {children}
                  </table>
                </div>
              ),
              thead: ({ children }) => (
                <thead className="bg-gray-50 dark:bg-gray-800">
                  {children}
                </thead>
              ),
              th: ({ children }) => (
                <th className="px-4 py-2 text-left text-sm font-semibold text-gray-900 dark:text-white">
                  {children}
                </th>
              ),
              td: ({ children }) => (
                <td className="px-4 py-2 text-sm text-gray-700 dark:text-gray-300">
                  {children}
                </td>
              ),
            }}
          >
            {chapter.content}
          </ReactMarkdown>
        </article>

        {/* Navigation Buttons */}
        <div className="flex items-center justify-between mt-12 pt-8 border-t border-gray-200 dark:border-gray-700">
          {prevChapter ? (
            <button
              onClick={() => navigate(`/chapter/${prevChapter.id}`)}
              className="btn-secondary inline-flex items-center gap-2 group"
            >
              <ChevronLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
              <div className="text-left">
                <div className="text-xs text-gray-500 dark:text-gray-400">Previous</div>
                <div className="font-medium">{prevChapter.title}</div>
              </div>
            </button>
          ) : (
            <div />
          )}

          {nextChapter ? (
            <button
              onClick={() => navigate(`/chapter/${nextChapter.id}`)}
              className="btn-primary inline-flex items-center gap-2 group ml-auto"
            >
              <div className="text-right">
                <div className="text-xs text-white/80">Next</div>
                <div className="font-medium">{nextChapter.title}</div>
              </div>
              <ChevronRight className="w-4 h-4 group-hover:translate-x-1 transition-transform" />
            </button>
          ) : (
            <button
              onClick={() => navigate('/')}
              className="btn-primary inline-flex items-center gap-2 ml-auto"
            >
              <Home className="w-4 h-4" />
              Back to Home
            </button>
          )}
        </div>
      </div>
    </div>
  );
};

export default ChapterReader;
