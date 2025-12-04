import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the gap between the digital brain and the physical body.',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-and-humanoid-robotics-textbook.panaversity.org',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'panaversity', // Usually your GitHub org/user name.
  projectName: 'physical-ai-and-humanoid-robotics-textbook', // Usually your repo name.

  // Please report any unexpected behavior in development mode to Docusaurus team.
  onBrokenLinks: 'warn',

  // Custom fields to be passed to the frontend
  customFields: {
    API_BASE_URL: process.env.API_BASE_URL || 'http://localhost:8000',
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // Added Urdu locale
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/panaversity/physical-ai-and-humanoid-robotics-textbook/tree/main/book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/panaversity/physical-ai-and-humanoid-robotics-textbook/tree/main/book/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          type: 'localeDropdown', // Added language dropdown
          position: 'right',
        },
        {
          href: 'https://github.com/panaversity/physical-ai-and-humanoid-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Book',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: Introduction to Physical AI',
              to: '/docs/module-0/01-foundations',
            },
            {
              label: 'Module 2: The Robotic Nervous System (ROS 2)',
              to: '/docs/module-1/ros-overview',
            },
            {
              label: 'Module 3: The Digital Twin (Gazebo & Unity)',
              to: '/docs/module-2/01-digital-twin-overview',
            },
            {
              label: 'Module 4: The AI-Robot Brain (NVIDIA Isaac™)',
              to: '/docs/module-3/01-nvidia-isaac-overview',
            },
            {
              label: 'Module 5: Vision-Language-Action (VLA)',
              to: '/docs/module-4/01-vla-overview',
            },
            {
              label: 'Module 6: Humanoid Robot Development',
              to: '/docs/module-5/01-kinematics-dynamics',
            },
            {
              label: 'Module 7: Conversational Robotics',
              to: '/docs/module-6/01-gpt-integration',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Panaverse Dao Discord',
              href: 'https://discord.gg/panaverse',
            },
            {
              label: 'YouTube',
              href: 'https://www.youtube.com/@panaverse',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/company/panaverse/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/panaversity/physical-ai-and-humanoid-robotics-textbook',
            },
            {
              label: 'Panaversity',
              href: 'https://www.panaversity.org/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} <a href="https://www.panaversity.org/" target="_blank" rel="noopener noreferrer" class="footer__link-item">Panaversity</a>. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
