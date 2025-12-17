# Quick Start Guide

Get your educational book running in 3 simple steps!

## 🚀 Quick Setup

### 1. Install Dependencies
```bash
cd book-app
npm install
```

### 2. Start Development Server
```bash
npm run dev
```

### 3. Open in Browser
Navigate to: **http://localhost:3000**

That's it! 🎉

## 📦 What's Included

- ✅ 4 comprehensive modules
- ✅ 12 detailed chapters
- ✅ Dark/Light mode
- ✅ Responsive design
- ✅ Search functionality
- ✅ Syntax-highlighted code blocks
- ✅ Beautiful typography

## 🎨 Features

### Homepage
- Professional landing page
- Module overview cards
- Course statistics

### Chapter Reading
- Clean, distraction-free reading experience
- Previous/Next navigation
- Breadcrumb navigation
- Markdown rendering with code highlighting

### Sidebar
- Collapsible module sections
- Chapter search
- Current chapter highlighting
- Mobile-friendly sliding menu

### Dark Mode
- Smooth theme transitions
- Persistent theme selection
- Beautiful color schemes for both modes

## 📱 Mobile Support

The app is fully responsive:
- **Desktop**: Fixed sidebar + main content
- **Tablet**: Collapsible sidebar
- **Mobile**: Sliding sidebar with hamburger menu

## 🛠️ Common Commands

```bash
# Development
npm run dev          # Start dev server

# Production
npm run build        # Build for production
npm run preview      # Preview production build

# Code Quality
npm run lint         # Run ESLint
```

## 🎯 Quick Tips

1. **Theme Toggle**: Click sun/moon icon in top-right
2. **Search**: Type in sidebar search box to filter chapters
3. **Navigation**: Use Previous/Next buttons at bottom of each chapter
4. **Sidebar**: Click hamburger menu on mobile to open/close

## 🐛 Troubleshooting

### Port Already in Use?
```bash
# Kill process on port 3000
lsof -ti:3000 | xargs kill -9

# Or use different port
npm run dev -- --port 3001
```

### Dependencies Not Installing?
```bash
# Clear npm cache
npm cache clean --force
rm -rf node_modules package-lock.json
npm install
```

### Build Errors?
```bash
# Clear Vite cache
rm -rf dist node_modules/.vite
npm install
npm run build
```

## 📚 Next Steps

1. Read the full **README.md** for detailed documentation
2. Explore the **src/data/bookContent.js** to see how content is structured
3. Customize colors in **tailwind.config.js**
4. Add your own chapters!

## 🤝 Need Help?

- Check the README.md for detailed docs
- Look at component files in `src/components/`
- Review the book content structure in `src/data/bookContent.js`

---

**Happy Reading! 📖**
