// @ts-check
/**
 * Contract: Correct Docusaurus Configuration for GitHub Pages Deployment
 *
 * Purpose: Template showing required configuration changes to fix routing and deployment
 *
 * Key Changes:
 * 1. Add routeBasePath: '/' to serve docs at site root (FR-003, FR-004)
 * 2. Update placeholder URLs with actual GitHub repository details (FR-005, FR-006, FR-007)
 * 3. Fix footer links after routeBasePath change
 *
 * Testing:
 * - Local: npm start → http://localhost:3000/book-ai/intro (should work)
 * - Build: npm run build → verify build/book-ai/intro/index.html exists
 * - Production: Deploy to GitHub Pages → verify https://[user].github.io/[repo]/ works
 */

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Textbook on Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',  // MUST exist at static/img/favicon.ico (FR-010)

  // GitHub Pages Configuration
  // CHANGE: Replace [USERNAME] and [REPO_NAME] with actual values (FR-005, FR-006)
  url: 'https://[USERNAME].github.io',           // e.g., 'https://johndoe.github.io'
  baseUrl: '/[REPO_NAME]/',                      // e.g., '/book-ai/' or '/Humanoid-Robotics-Textbook/'

  organizationName: '[USERNAME]',                 // e.g., 'johndoe'
  projectName: '[REPO_NAME]',                     // e.g., 'book-ai'

  onBrokenLinks: 'throw',                         // Fail build on broken links (catch 404s early)
  onBrokenMarkdownLinks: 'warn',                  // Warn on markdown link issues

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
          // KEY FIX: Serve docs at root path instead of /docs/ (FR-003, FR-004)
          routeBasePath: '/',                     // Changes URLs from /docs/intro to /intro

          sidebarPath: './sidebars.js',

          // CHANGE: Update with actual GitHub repository URL (FR-007)
          editUrl: 'https://github.com/[USERNAME]/[REPO_NAME]/tree/main/',
        },
        blog: false,                              // No blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',    // Social media preview (optional)
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Textbook Logo',
          src: 'img/logo.svg',                    // MUST exist at static/img/logo.svg (FR-010)
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',         // References sidebars.js textbookSidebar
            position: 'left',
            label: 'Textbook',                    // Navbar link text (FR-007)
          },
          {
            // CHANGE: Update with actual GitHub repository URL (FR-007)
            href: 'https://github.com/[USERNAME]/[REPO_NAME]',
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
                // FIX: Remove /docs/ prefix after routeBasePath: '/' change
                to: '/module-1-ros2/index',       // Changed from '/docs/module-1-ros2'
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/module-2-digital-twin/index',  // New link for Module 2
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
                // CHANGE: Update with actual GitHub repository URL
                label: 'GitHub',
                href: 'https://github.com/[USERNAME]/[REPO_NAME]',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'cpp', 'xml'],  // Added cpp, xml for URDF/SDF
      },
    }),
};

export default config;

/**
 * Validation Checklist:
 *
 * - [ ] url updated with actual GitHub username
 * - [ ] baseUrl updated with actual repository name
 * - [ ] organizationName updated with GitHub username
 * - [ ] projectName updated with repository name
 * - [ ] routeBasePath set to '/' in docs preset
 * - [ ] editUrl updated with actual repository URL
 * - [ ] navbar GitHub href updated
 * - [ ] footer GitHub href updated
 * - [ ] footer module links updated (removed /docs/ prefix)
 * - [ ] static/img/ directory exists with favicon and logo
 *
 * Testing Commands:
 *
 * 1. npm install           # Install dependencies
 * 2. npm start             # Test local dev at http://localhost:3000/[baseUrl]/intro
 * 3. npm run build         # Test production build
 * 4. Deploy to GitHub Pages and verify navigation works
 */
