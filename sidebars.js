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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/index',
          label: 'Module Overview',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/ros2-fundamentals',
          label: 'Chapter 1: ROS 2 Fundamentals',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/python-agents-ros',
          label: 'Chapter 2: Python Agents & ROS Integration',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/urdf-humanoids',
          label: 'Chapter 3: URDF for Humanoids',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/lab-building-ros2-robot',
          label: 'Chapter 4: Lab - Building a ROS 2 Robot',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/references',
          label: 'References',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-2-digital-twin/index',
          label: 'Module Overview',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/gazebo-physics',
          label: 'Chapter 1: Gazebo Physics Simulation',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/unity-visualization',
          label: 'Chapter 2: Unity for High-Fidelity Simulation',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/sensor-simulation',
          label: 'Chapter 3: Simulating Sensors',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/lab-digital-twin',
          label: 'Chapter 4: Lab - Building a Digital Twin',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/references',
          label: 'References',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-3-ai-robot-brain/index',
          label: 'Module Overview',
        },
        {
          type: 'doc',
          id: 'module-3-ai-robot-brain/intro',
          label: 'Chapter 1: Introduction to AI-Driven Robotics',
        },
        {
          type: 'doc',
          id: 'module-3-ai-robot-brain/isaac-sim',
          label: 'Chapter 2: Isaac Sim Fundamentals',
        },
        {
          type: 'doc',
          id: 'module-3-ai-robot-brain/isaac-ros-perception',
          label: 'Chapter 3: Isaac ROS for Perception',
        },
        {
          type: 'doc',
          id: 'module-3-ai-robot-brain/navigation-and-sim2real',
          label: 'Chapter 4: Navigation and Sim-to-Real Transfer',
        },
        {
          type: 'doc',
          id: 'module-3-ai-robot-brain/references',
          label: 'References',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-4-vla/index',
          label: 'Module Overview',
        },
        {
          type: 'doc',
          id: 'module-4-vla/intro-vla',
          label: 'Chapter 1: Introduction to VLA',
        },
        {
          type: 'doc',
          id: 'module-4-vla/voice-to-action',
          label: 'Chapter 2: Voice-to-Action Pipeline',
        },
        {
          type: 'doc',
          id: 'module-4-vla/language-planning',
          label: 'Chapter 3: Language-to-Plan with LLMs',
        },
        {
          type: 'doc',
          id: 'module-4-vla/capstone-autonomous-humanoid',
          label: 'Chapter 4: Vision-Guided Action and Capstone',
        },
        {
          type: 'doc',
          id: 'module-4-vla/references',
          label: 'References',
        },
      ],
    },
  ],
};

export default sidebars;
