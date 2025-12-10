// @ts-check

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro', // 'intro' is the first item, linking to docs/intro.md
    {
      type: 'category',
      label: 'Chapter 1: Introduction', // New category for moved sub-chapters
      link: {
        type: 'generated-index', // Using generated-index for this category
        title: 'Chapter 1 Overview',
        slug: '/category/chapter-1-introduction',
      },
      items: [
        'chapter-1-introduction/what-is-a-physical-ai',
        'chapter-1-introduction/the-brains-of-your-robot',
        'chapter-1-introduction/giving-your-robot-a-body',
      ],
    },
    {
      type: 'category',
      label: 'Constitution',
      link: {
        type: 'doc',
        id: 'Constitution/constitution',
      },
      items: [
        'Constitution/constitution',
      ],
    },
    {
      type: 'category',
      label: 'Specification',
      link: {
        type: 'doc',
        id: 'Specification/specification',
      },
      items: [
        'Specification/specification',
      ],
    },
    {
      type: 'category',
      label: 'Plan',
      link: {
        type: 'doc',
        id: 'Plan/plan',
      },
      items: [
        'Plan/plan',
      ],
    },
    {
      type: 'category',
      label: 'Tasks',
      link: {
        type: 'doc',
        id: 'Tasks/tasks',
      },
      items: [
        'Tasks/tasks',
      ],
    },
    {
      type: 'category',
      label: 'Implementation',
      link: {
        type: 'doc',
        id: 'Implementation/implementation',
      },
      items: [
        'Implementation/implementation',
      ],
    },
  ],
};

export default sidebars;