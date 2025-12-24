// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/**
 * Environment detect karne ke liye
 * true = Vercel deploy
 * false = Local / GitHub Pages
 */
const isVercel = process.env.VERCEL === '1';

/** @type {import('@docusaurus/types').Config} */


const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Textbook on Physical AI and Humanoid Robotics',
  // IMPORTANT: Update favicon reference to match created file
  // We created favicon.svg instead of favicon.ico (modern browsers support SVG)
  favicon: 'img/favicon.svg',

  // Set the production url of your site here
  // TODO: Replace 'your-org' with actual GitHub username
  // url: 'https://your-org.github.io',
  // url: 'https://hackathon-1-create-a-textbook-whe5.vercel.app',

  // Production URL
  url: isVercel
    ? 'https://hackathon-1-create-a-textbook-whe5.vercel.app' // Vercel URL
    : 'https://your-org.github.io', // GitHub Pages URL

  baseUrl: isVercel ? '/' : '/book-ai/',

  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  // This MUST match your GitHub repository name exactly
  // Current: '/book-ai/' - update if repository name is different
  // baseUrl: '/book-ai/', // for git pages deploy.

  
    // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  // TODO: Replace 'your-org' with actual GitHub username
  organizationName: 'your-org', // Usually your GitHub org/user name.
  // TODO: Update if repository name is different from 'book-ai'
  projectName: 'book-ai', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          // KEY FIX: Serve docs at root path instead of /docs/ prefix
          // This eliminates the need for /docs/ in URLs (e.g., /intro instead of /docs/intro)
          routeBasePath: '/',    // Serve docs at root
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/book-ai/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Textbook Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/your-org/book-ai',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: ROS 2',
                // FIXED: Removed /docs/ prefix and /index suffix (Docusaurus auto-resolves to index page)
                to: '/module-1-ros2/',
              },
              {
                label: 'Module 2: Digital Twin',
                // FIXED: Removed /docs/ prefix and /index suffix (Docusaurus auto-resolves to index page)
                to: '/module-2-digital-twin/',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/book-ai',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash'],
      },
    }),
};

export default config;
