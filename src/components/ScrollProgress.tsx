import React, { useEffect, useState } from 'react';
import styles from './ScrollProgress.module.css';

const ScrollProgress: React.FC = () => {
  const [scrollProgress, setScrollProgress] = useState(0);

  useEffect(() => {
    const updateScrollProgress = () => {
      const scrollPx = document.documentElement.scrollTop;
      const winHeightPx =
        document.documentElement.scrollHeight -
        document.documentElement.clientHeight;
      const scrolled = (scrollPx / winHeightPx) * 100;

      setScrollProgress(scrolled);
    };

    // Update on mount
    updateScrollProgress();

    // Add event listener
    window.addEventListener('scroll', updateScrollProgress);

    // Cleanup
    return () => window.removeEventListener('scroll', updateScrollProgress);
  }, []);

  // Only show on doc pages
  if (typeof window === 'undefined') return null;

  const isDocPage = window.location.pathname.includes('/docs/');
  if (!isDocPage) return null;

  return (
    <div className={styles.progressContainer}>
      <div
        className={styles.progressBar}
        style={{ width: `${scrollProgress}%` }}
      />
    </div>
  );
};

export default ScrollProgress;
