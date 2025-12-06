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
        id: 'module-1-ros2/index',
      },
      collapsed: true,
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
      label: 'Module 2: Simulation',
      link: {
        type: 'doc',
        id: 'module-2-simulation/index',
      },
      collapsed: true,
      items: [
        'module-2-simulation/2-1-gazebo-intro',
        'module-2-simulation/2-2-unity-intro',
        'module-2-simulation/2-3-sensors',
        'module-2-simulation/2-4-world-building',
        {
          type: 'category',
          label: 'Exercises',
          items: [
            'module-2-simulation/exercises/module-2-exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'doc',
        id: 'module-3-isaac/index',
      },
      collapsed: true,
      items: [
        'module-3-isaac/3-1-isaac-sim-intro',
        'module-3-isaac/3-2-reinforcement-learning',
        'module-3-isaac/3-3-training-scenarios',
        {
          type: 'category',
          label: 'Exercises',
          items: [
            'module-3-isaac/exercises/module-3-exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module-4-vla/index',
      },
      collapsed: true,
      items: [
        'module-4-vla/4-1-vision-pipelines',
        'module-4-vla/4-2-language-models',
        'module-4-vla/4-3-action-coordination',
        {
          type: 'category',
          label: 'Exercises',
          items: [
            'module-4-vla/exercises/module-4-exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'doc',
        id: 'capstone-index',
      },
      collapsed: false,
      items: [
        'capstone-requirements',
        'capstone-milestones',
        'capstone-evaluation',
        'capstone-troubleshooting',
      ],
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
