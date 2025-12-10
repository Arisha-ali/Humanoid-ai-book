# Book Specification: My First AI Humanoid

**Feature**: physical-ai-book
**Created**: 2025-12-10
**Status**: Draft

## 1. Book Structure

This book is organized into chapters and lessons. Each lesson is a separate markdown file.

### Chapter 1: Introduction to Physical AI

*   **Description**: This chapter introduces the fundamental concepts of physical AI, robotics, and the components needed to build a simple humanoid robot.

*   **Lessons**:
    1.  **Lesson 1: What is a Physical AI?**
        *   **Description**: A gentle introduction to the world of AI and robotics, explaining the difference between software AI and physical AI. It will also showcase some exciting examples of physical AI in the real world.
    2.  **Lesson 2: The Brains of Your Robot**
        *   **Description**: This lesson will introduce the microcontroller (e.g., Arduino or Raspberry Pi) as the "brain" of the robot. It will explain what a microcontroller is, what it does, and why it's essential for our project.
    3.  **Lesson 3: Giving Your Robot a Body**
        *   **Description**: This lesson will cover the basic components for building a robot's chassis, including motors, sensors, and power supply. It will provide a simple, easy-to-assemble design.

## 2. Content Guidelines and Lesson Format

### Content Guidelines

*   **Target Audience**: Beginners with no prior knowledge of AI or robotics.
*   **Tone**: Encouraging, inspiring, and friendly.
*   **Language**: Simple, clear, and concise. Avoid jargon where possible.
*   **Visuals**: Use plenty of diagrams, illustrations, and photos.
*   **Code**: All code must be well-documented and tested.

### Lesson Format

Each lesson will be a markdown file (`.md` or `.mdx`) and should follow this structure:

1.  **Title**: Clear and descriptive.
2.  **Introduction**: A brief overview of what the lesson will cover.
3.  **Learning Objectives**: A list of what the reader will be able to do after completing the lesson.
4.  **Main Content**: The core of the lesson, broken down into easy-to-digest sections with headings and subheadings.
5.  **Hands-on Project**: A practical, step-by-step project that allows the reader to apply what they've learned.
6.  **Summary**: A recap of the key takeaways.
7.  **Next Steps**: A preview of the next lesson.

## 3. Docusaurus Specific Requirements

### Organisation

*   All book content will reside in the `docs` directory.
*   Each chapter will be a subdirectory within `docs`.
*   Each lesson will be a markdown file within its chapter's subdirectory.
*   A `_category_.json` file will be used in each chapter directory to define the chapter's title and its position in the sidebar.

### Example Structure:

```
docs/
├── intro.md
└── chapter-1/
    ├── _category_.json
    ├── lesson-1.md
    ├── lesson-2.md
    └── lesson-3.md
```

### Sidebar

The `sidebars.js` file will be configured to automatically generate the sidebar from the `docs` directory structure.

### Markdown Features

We will leverage Docusaurus's enhanced markdown features, such as:
*   **Admonitions**: for notes, tips, and warnings.
*   **Code blocks with syntax highlighting**: for all code examples.
*   **MDX**: to include interactive React components where necessary.
