/**
 * Contract: Sidebar Configuration with Correct ID Patterns
 *
 * Purpose: Template showing correct sidebar ID naming convention
 *
 * Key Rules:
 * 1. Sidebar IDs should be simplified (no numeric prefixes) - FR-009
 * 2. IDs use forward slashes for nested paths: 'module-name/chapter-slug'
 * 3. Filenames can have numeric prefixes (01-), but IDs should not
 * 4. Labels display user-friendly text with chapter numbers
 *
 * Example Mapping:
 * - File: docs/module-2-digital-twin/01-gazebo-physics.md
 * - ID: 'module-2-digital-twin/gazebo-physics' (no 01- prefix)
 * - Label: 'Chapter 1: Gazebo Physics Simulation'
 *
 * Benefits:
 * - Renumbering files doesn't require sidebar updates
 * - IDs are semantic and maintainable
 * - Decouples content hierarchy from file naming
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    // === Introduction ===
    {
      type: 'doc',
      id: 'intro',                                    // File: docs/intro.md
      label: 'Introduction',
    },

    // === Module 1: ROS 2 ===
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,                               // Start expanded
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/index',                  // File: docs/module-1-ros2/index.md
          label: 'Module Overview',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/ros2-fundamentals',      // File: docs/module-1-ros2/01-ros2-fundamentals.md
          label: 'Chapter 1: ROS 2 Fundamentals',     // ID has no '01-' prefix
        },
        {
          type: 'doc',
          id: 'module-1-ros2/python-agents-ros',      // File: docs/module-1-ros2/02-python-agents-ros.md
          label: 'Chapter 2: Python Agents & ROS Integration',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/urdf-humanoids',         // File: docs/module-1-ros2/03-urdf-humanoids.md
          label: 'Chapter 3: URDF for Humanoids',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/lab-building-ros2-robot', // File: docs/module-1-ros2/04-lab-building-ros2-robot.md
          label: 'Chapter 4: Lab - Building a ROS 2 Robot',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/references',             // File: docs/module-1-ros2/references.md
          label: 'References',
        },
      ],
    },

    // === Module 2: Digital Twin ===
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-2-digital-twin/index',          // File: docs/module-2-digital-twin/index.md
          label: 'Module Overview',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/gazebo-physics', // File: docs/module-2-digital-twin/01-gazebo-physics.md
          label: 'Chapter 1: Gazebo Physics Simulation',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/unity-visualization', // File: docs/module-2-digital-twin/02-unity-visualization.md
          label: 'Chapter 2: Unity for High-Fidelity Simulation',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/sensor-simulation',   // File: docs/module-2-digital-twin/03-sensor-simulation.md
          label: 'Chapter 3: Simulating Sensors',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/lab-digital-twin',    // File: docs/module-2-digital-twin/04-lab-digital-twin.md
          label: 'Chapter 4: Lab - Building a Digital Twin',
        },
        {
          type: 'doc',
          id: 'module-2-digital-twin/references',          // File: docs/module-2-digital-twin/references.md
          label: 'References',
        },
      ],
    },
  ],
};

export default sidebars;

/**
 * Validation Rules:
 *
 * 1. ID Pattern: 'module-folder/file-slug' (no numeric prefixes like 01-)
 * 2. File Mapping: Docusaurus automatically strips numeric prefixes
 *    - File: 01-chapter.md → ID: chapter (derived)
 *    - Or specify ID in frontmatter: `id: chapter`
 * 3. Nested Paths: Use forward slash / not hyphen -
 *    - Correct: 'module-2-digital-twin/gazebo-physics'
 *    - Wrong: 'module-2-digital-twin-gazebo-physics'
 * 4. All IDs must have corresponding markdown files in docs/ directory
 * 5. Category items must not be empty (Docusaurus build fails)
 *
 * Testing:
 * - Run npm start and verify all sidebar links work
 * - Click each sidebar item and confirm page loads
 * - No 404 errors or "Page not found" messages
 */
