/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      link: {
        type: 'doc',
        id: 'module-1-ros2/module-1-ros2-index',
      },
      collapsed: false,
      items: [
        'module-1-ros2/1-1-ros2-intro',
        'module-1-ros2/1-2-nodes-topics',
        'module-1-ros2/1-3-services-actions',
        'module-1-ros2/1-4-parameters',
        'module-1-ros2/1-5-urdf',
        {
          type: 'category',
          label: 'Exercises',
          items: [
            'module-1-ros2/exercises/module-1-exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity',
      link: {
        type: 'doc',
        id: 'module-2-simulation/module-2-simulation-index',
      },
      collapsed: true,
      items: [],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'doc',
        id: 'module-3-isaac/module-3-isaac-index',
      },
      collapsed: true,
      items: [],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module-4-vla/module-4-vla-index',
      },
      collapsed: true,
      items: [],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'doc',
        id: 'capstone/capstone-index',
      },
      collapsed: true,
      items: [],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: true,
      items: [
        'resources/glossary',
        'resources/hardware-specs',
        'resources/software-setup',
        'resources/further-reading',
      ],
    },
  ],
};

module.exports = sidebars;
