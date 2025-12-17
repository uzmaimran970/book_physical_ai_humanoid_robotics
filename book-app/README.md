# Physical AI & Humanoid Robotics - Interactive Book

A modern, responsive educational book built with React and Tailwind CSS. Features include dark/light mode, mobile-friendly navigation, chapter search, and beautiful typography.

## Features

- **Professional Design**: Clean, modern UI with gradient accents
- **Dark/Light Mode**: Toggle between themes with smooth transitions
- **Responsive**: Mobile-first design with sliding sidebar on small screens
- **Search**: Quickly find chapters by keyword
- **Navigation**: Previous/Next chapter buttons for easy reading flow
- **Syntax Highlighting**: Beautiful code blocks with syntax highlighting
- **12 Comprehensive Chapters**: Covering ROS 2, Digital Twins, NVIDIA Isaac, and VLA models

## Tech Stack

- **React 18**: Modern React with hooks
- **React Router v6**: Client-side routing
- **Tailwind CSS**: Utility-first CSS framework
- **Vite**: Fast build tool and dev server
- **React Markdown**: Markdown rendering with custom components
- **Prism**: Syntax highlighting for code blocks
- **Lucide React**: Beautiful, consistent icons

## Installation

### Prerequisites

- Node.js 18+ and npm

### Steps

1. **Navigate to the project directory**:
   ```bash
   cd book-app
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Install additional required packages** (if not auto-installed):
   ```bash
   npm install react-markdown remark-gfm react-syntax-highlighter @tailwindcss/typography
   ```

4. **Start development server**:
   ```bash
   npm run dev
   ```

5. **Open your browser**:
   Navigate to `http://localhost:3000`

## Build for Production

```bash
npm run build
```

The optimized build will be in the `dist/` folder.

### Preview production build:
```bash
npm run preview
```

## Project Structure

```
book-app/
├── src/
│   ├── components/
│   │   ├── HomePage.jsx          # Landing page with module overview
│   │   ├── Sidebar.jsx           # Navigation sidebar with search
│   │   ├── ChapterReader.jsx     # Chapter content viewer
│   │   └── ThemeToggle.jsx       # Dark/light mode toggle
│   ├── data/
│   │   └── bookContent.js        # All book content and data
│   ├── App.jsx                   # Main app component
│   ├── main.jsx                  # Entry point
│   └── index.css                 # Global styles and Tailwind imports
├── index.html                    # HTML template
├── tailwind.config.js            # Tailwind configuration
├── vite.config.js                # Vite configuration
└── package.json                  # Dependencies and scripts
```

## Content Structure

The book contains **4 modules** with **12 chapters**:

### Module 1: The Robotic Nervous System (ROS 2)
1. ROS 2 Nodes and Topics
2. ROS 2 Services and Actions
3. ROS 2 Launch Files and Parameters

### Module 2: The Digital Twin (Gazebo & Unity)
1. Introduction to Digital Twins
2. Gazebo Simulation
3. Unity Simulation and Robotics

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
1. Introduction to NVIDIA Isaac
2. Isaac Navigation Stack
3. Isaac Perception and Manipulation

### Module 4: Vision-Language-Action (VLA)
1. Vision-Language Models for Robotics
2. Language-Driven Robot Control
3. End-to-End VLA Systems

## Customization

### Changing Colors

Edit `tailwind.config.js` to modify the color scheme:

```javascript
theme: {
  extend: {
    colors: {
      primary: { /* your colors */ },
      accent: { /* your colors */ }
    }
  }
}
```

### Adding Content

Edit `src/data/bookContent.js` to add or modify chapters:

```javascript
{
  id: "m1-c4",
  moduleId: "module-1",
  number: 4,
  title: "Your New Chapter",
  content: `
# Chapter Title

Your markdown content here...
  `
}
```

### Styling

- Global styles: `src/index.css`
- Component styles: Inline Tailwind classes
- Custom utilities: Add to `@layer utilities` in `index.css`

## Browser Support

- Chrome (latest)
- Firefox (latest)
- Safari (latest)
- Edge (latest)

## Performance

- Lazy loading for images
- Code splitting with React Router
- Optimized production builds with Vite
- Fast development with HMR (Hot Module Replacement)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

MIT License - feel free to use this project for educational purposes.

## Credits

Built with ❤️ for robotics education.

## Support

For issues or questions, please open an issue on GitHub.

---

**Happy Learning! 🤖**
# Force Vercel redeploy - Wed Dec 17 21:46:30 PKT 2025
