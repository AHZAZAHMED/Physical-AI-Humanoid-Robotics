/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction & Foundations',
      items: [
        'intro',
        'foundations-of-physical-ai',
        'humanoid-robotics-overview',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1/ros-2-architecture',
        'module-1/nodes-topics-and-services',
        'module-1/urdf-and-packages',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'module-2/gazebo-setup-and-physics',
        'module-2/unity-and-high-fidelity-rendering',
        'module-2/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      items: [
        'module-3/isaac-sim-and-sdk',
        'module-3/isaac-ros-and-vslam',
        'module-3/nav2-and-bipedal-movement',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Humanoid Development',
      items: [
        'module-4/voice-to-action',
        'module-4/cognitive-planning',
        'module-4/humanoid-kinematics-and-control',
      ],
    },
    {
      type: 'category',
      label: 'Assessments & Hardware',
      items: [
        'assessments/hardware-requirements',
        'assessments/capstone-project-definition',
      ],
    },
  ],
};

module.exports = sidebars;