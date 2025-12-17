#!/bin/bash

# Physical AI & Humanoid Robotics Book - Setup Script
# This script automates the initial setup process

set -e  # Exit on error

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  Physical AI & Humanoid Robotics Book - Setup Script         ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed!"
    echo "📥 Please install Node.js 18+ from: https://nodejs.org/"
    exit 1
fi

# Check Node.js version
NODE_VERSION=$(node -v | cut -d'v' -f2 | cut -d'.' -f1)
if [ "$NODE_VERSION" -lt 18 ]; then
    echo "⚠️  Node.js version is too old (found v$NODE_VERSION)"
    echo "📥 Please upgrade to Node.js 18+ from: https://nodejs.org/"
    exit 1
fi

echo "✅ Node.js $(node -v) detected"
echo ""

# Check if npm is installed
if ! command -v npm &> /dev/null; then
    echo "❌ npm is not installed!"
    exit 1
fi

echo "✅ npm $(npm -v) detected"
echo ""

# Install dependencies
echo "📦 Installing dependencies..."
echo "   This may take a few minutes..."
echo ""

npm install

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Dependencies installed successfully!"
else
    echo ""
    echo "❌ Failed to install dependencies"
    exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  🎉 Setup Complete!                                           ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "📚 Your book is ready to run!"
echo ""
echo "To start the development server:"
echo "  $ npm run dev"
echo ""
echo "Then open your browser to: http://localhost:3000"
echo ""
echo "Other available commands:"
echo "  $ npm run build    # Build for production"
echo "  $ npm run preview  # Preview production build"
echo "  $ npm run lint     # Run code linter"
echo ""
echo "📖 For more information, see:"
echo "  - README.md         (Full documentation)"
echo "  - QUICK_START.md    (Quick start guide)"
echo "  - DEPLOYMENT.md     (Deployment instructions)"
echo ""
echo "Happy learning! 🤖"
