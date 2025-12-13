/**
 * Validation Script: Verify Total Reading Time < 45 minutes
 * Reads all chapter markdown files and calculates total reading time
 */

const fs = require('fs');
const path = require('path');

const DOCS_DIR = path.join(__dirname, '..', 'docs');
const MAX_READING_TIME_MINUTES = 45;

/**
 * Extract front matter from markdown file
 */
function extractFrontMatter(content) {
  const frontMatterRegex = /^---\n([\s\S]*?)\n---/;
  const match = content.match(frontMatterRegex);

  if (!match) {
    return null;
  }

  const frontMatter = {};
  const lines = match[1].split('\n');

  for (const line of lines) {
    const [key, ...valueParts] = line.split(':');
    if (key && valueParts.length) {
      const value = valueParts.join(':').trim();
      // Remove quotes if present
      frontMatter[key.trim()] = value.replace(/^["']|["']$/g, '');
    }
  }

  return frontMatter;
}

/**
 * Get all chapter files
 */
function getChapterFiles() {
  const files = fs.readdirSync(DOCS_DIR);
  return files
    .filter(file => file.startsWith('chapter-') && file.endsWith('.md'))
    .map(file => path.join(DOCS_DIR, file));
}

/**
 * Validate reading time for all chapters
 */
function validateReadingTime() {
  console.log('='.repeat(60));
  console.log('READING TIME VALIDATION');
  console.log('='.repeat(60));
  console.log();

  const chapterFiles = getChapterFiles();

  if (chapterFiles.length === 0) {
    console.error('❌ No chapter files found!');
    process.exit(1);
  }

  console.log(`Found ${chapterFiles.length} chapter files\n`);

  let totalReadingTime = 0;
  const chapters = [];

  for (const filePath of chapterFiles) {
    const content = fs.readFileSync(filePath, 'utf-8');
    const frontMatter = extractFrontMatter(content);

    if (!frontMatter) {
      console.error(`❌ No front matter found in ${path.basename(filePath)}`);
      continue;
    }

    const readingTime = parseInt(frontMatter.reading_time || 0);
    const title = frontMatter.title || path.basename(filePath, '.md');
    const difficulty = frontMatter.difficulty || 'unknown';

    if (!readingTime) {
      console.error(`❌ No reading_time in ${path.basename(filePath)}`);
      continue;
    }

    chapters.push({
      file: path.basename(filePath),
      title,
      readingTime,
      difficulty
    });

    totalReadingTime += readingTime;

    console.log(`✓ ${path.basename(filePath)}`);
    console.log(`  Title: ${title}`);
    console.log(`  Reading Time: ${readingTime} minutes`);
    console.log(`  Difficulty: ${difficulty}`);
    console.log();
  }

  console.log('='.repeat(60));
  console.log('SUMMARY');
  console.log('='.repeat(60));
  console.log();
  console.log(`Total Chapters: ${chapters.length}`);
  console.log(`Total Reading Time: ${totalReadingTime} minutes`);
  console.log(`Maximum Allowed: ${MAX_READING_TIME_MINUTES} minutes`);
  console.log();

  // Difficulty breakdown
  const difficultyBreakdown = chapters.reduce((acc, chapter) => {
    acc[chapter.difficulty] = (acc[chapter.difficulty] || 0) + 1;
    return acc;
  }, {});

  console.log('Difficulty Distribution:');
  for (const [difficulty, count] of Object.entries(difficultyBreakdown)) {
    console.log(`  ${difficulty}: ${count} chapters`);
  }
  console.log();

  // Validation result
  if (totalReadingTime <= MAX_READING_TIME_MINUTES) {
    console.log(`✅ PASS: Total reading time (${totalReadingTime} min) is within the limit (${MAX_READING_TIME_MINUTES} min)`);
    console.log(`   Margin: ${MAX_READING_TIME_MINUTES - totalReadingTime} minutes remaining`);
    return true;
  } else {
    console.log(`❌ FAIL: Total reading time (${totalReadingTime} min) exceeds the limit (${MAX_READING_TIME_MINUTES} min)`);
    console.log(`   Overage: ${totalReadingTime - MAX_READING_TIME_MINUTES} minutes`);
    process.exit(1);
  }
}

// Run validation
try {
  validateReadingTime();
} catch (error) {
  console.error('❌ Error during validation:', error.message);
  process.exit(1);
}
