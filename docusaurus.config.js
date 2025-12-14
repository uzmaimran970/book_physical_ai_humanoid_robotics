// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const {themes} = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Interactive Textbook',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-test-site.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'your-org',
  projectName: 'ai-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization config
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Remove this to remove the "edit this page" links.
          editUrl: undefined,
        },
        blog: false, // Disable blog feature
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        hideOnScroll: false,
        logo: {
          alt: 'AI Textbook Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: '📚 Docs',
          },
          {
            to: '/',
            label: '🏠 Home',
            position: 'left',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            type: 'search',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'All Modules',
                to: '/',
              },
            ],
          },
          {
            title: 'Learning Path',
            items: [
              {
                label: 'ROS 2 (Module 1)',
                to: '/docs/intro',
              },
              {
                label: 'Digital Twins (Module 2)',
                to: '/docs/intro',
              },
              {
                label: 'NVIDIA Isaac (Module 3)',
                to: '/docs/intro',
              },
              {
                label: 'VLA Models (Module 4)',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/ai-textbook',
              },
              {
                label: 'Documentation',
                to: '/docs/intro',
              },
            ],
          },
        ],
        logo: {
          alt: 'Physical AI & Humanoid Robotics',
          src: 'img/logo.svg',
          width: 40,
          height: 40,
        },
        copyright: `
          <div style="margin-top: 1.5rem; padding-top: 1.5rem; border-top: 1px solid rgba(250, 204, 21, 0.15);">
            <p style="margin: 0; font-size: 14px;">
              <strong style="color: #facc15;">Physical AI & Humanoid Robotics</strong>
            </p>
            <p style="margin: 0.5rem 0 0 0; font-size: 13px; color: #94a3b8;">
              An AI-Native Interactive Textbook
            </p>
            <p style="margin: 1rem 0 0 0; font-size: 12px; color: #64748b;">
              Copyright © ${new Date().getFullYear()} • Built with Docusaurus
            </p>
          </div>
        `,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'json'],
      },
    }),
};

module.exports = config;
