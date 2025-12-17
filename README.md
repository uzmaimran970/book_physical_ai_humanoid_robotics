# Physical AI & Humanoid Robotics - Interactive Textbook

A static interactive textbook for Physical AI & Humanoid Robotics built with Docusaurus. Features multilingual support (English/Urdu) and example UI components for quizzes, chat, and personalization.

## Features

- 📚 **8 Comprehensive Chapters** - Complete coverage of Physical AI & Humanoid Robotics
- 🌐 **Bilingual Support** - English and Urdu (اردو) with RTL support
- 📱 **Responsive Design** - Mobile-friendly reading experience
- 🎨 **Example UI Components** - Non-functional examples of:
  - Interactive quizzes with sample questions
  - RAG chatbot interface
  - Authentication forms
  - Personalization settings

> **Note:** All backend-dependent features (RAG chatbot, authentication, quizzes submission) are disabled. The UI components are provided as examples for demonstration purposes only.

## Tech Stack

- **Docusaurus 3.x** - Static site generation and documentation
- **React 18** - UI components
- **TypeScript** - Type safety
- **i18n** - Internationalization with RTL support for Urdu

## Quick Start

### Prerequisites
- Node.js 20+ (or 18+)
- npm or yarn

### Installation & Development

```bash
# Navigate to the website directory
cd website

# Install dependencies
npm install

# Start development server
npm start
```

The site will be available at: http://localhost:3000

### Build for Production

```bash
cd website

# Build static site
npm run build

# Serve production build locally
npm run serve
```

The production build will be in `website/build/` directory.

## Project Structure

```
hackathon_2025/
├── website/                    # Docusaurus site
│   ├── docs/                  # Markdown chapter content
│   │   ├── intro.md
│   │   ├── chapter-1.md       # Introduction to Physical AI
│   │   ├── chapter-2.md       # Humanoid Robot Anatomy
│   │   ├── chapter-3.md       # Sensors & Perception
│   │   ├── chapter-4.md       # Locomotion & Balance
│   │   ├── chapter-5.md       # Control Systems
│   │   ├── chapter-6.md       # Machine Learning for Robotics
│   │   ├── chapter-7.md       # Sim-to-Real Transfer
│   │   └── chapter-8.md       # Future of Physical AI
│   ├── src/
│   │   ├── components/        # React UI components
│   │   │   ├── ChatWidget.tsx         # Example chat UI
│   │   │   ├── ChapterQuiz.tsx        # Sample quiz component
│   │   │   ├── Login.tsx              # Example auth forms
│   │   │   ├── SignUp.tsx
│   │   │   ├── PersonalizationSettings.tsx
│   │   │   └── RecommendationsWidget.tsx
│   │   ├── contexts/          # React contexts
│   │   │   └── AuthContext.tsx
│   │   ├── pages/             # Custom pages
│   │   └── css/               # Styling
│   ├── i18n/                  # Urdu translations
│   │   └── ur/
│   │       └── docusaurus-plugin-content-docs/
│   ├── docusaurus.config.js   # Site configuration
│   ├── sidebars.js            # Sidebar structure
│   └── package.json           # Dependencies
├── my-website/                # Alternative Docusaurus setup
└── README.md                  # This file
```

## Content

The textbook covers 8 chapters on Physical AI and Humanoid Robotics:

1. **Introduction to Physical AI** - Overview and key concepts
2. **Humanoid Robot Anatomy** - Hardware components and design
3. **Sensors & Perception** - Vision, IMU, force sensors
4. **Locomotion & Balance** - Walking, ZMP, dynamic stability
5. **Control Systems** - Trajectory planning, inverse kinematics
6. **Machine Learning for Robotics** - Deep RL, imitation learning
7. **Sim-to-Real Transfer** - Domain randomization, reality gap
8. **Future of Physical AI** - Trends and ethical considerations

## Deployment

### Static Hosting (Recommended)

Deploy to any static hosting platform:

**Vercel:**
```bash
cd website
npx vercel
```

**Netlify:**
```bash
cd website
npm run build
# Upload build/ folder to Netlify
```

**GitHub Pages:**
```bash
cd website
npm run deploy
```

## Environment Variables

No environment variables are required for the static site. All configuration is in `website/docusaurus.config.js`.

## License

MIT

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.
