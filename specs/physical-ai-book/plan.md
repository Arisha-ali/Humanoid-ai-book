# Development Plan: My First AI Humanoid

**Feature**: physical-ai-book
**Date**: 2025-12-10
**Spec**: [specs/physical-ai-book/spec.md](spec.md)

## 1. Docusaurus Setup and Configuration

### Phase 1: Initial Setup

1.  **Initialize Docusaurus**: Use the `npx create-docusaurus@latest` command to scaffold a new Docusaurus project.
2.  **Configure `docusaurus.config.js`**:
    *   Set the `title`, `tagline`, and `url`.
    *   Configure the `presets` for docs.
    *   Customize the navbar and footer.
3.  **Customize Styling**: Modify `src/css/custom.css` to match the book's branding.

### Phase 2: Sidebar Configuration

1.  **Organize Docs**: Create the initial directory structure for chapters and lessons inside the `docs` directory as defined in the spec.
2.  **Configure `sidebars.js`**: Set up the sidebar to automatically generate from the `docs` directory structure. Use `_category_.json` files to manage chapter titles and order.

## 2. Content Development Phases

### Phase 1: Chapter 1 - Introduction to Physical AI

1.  **Write Lesson 1**: "What is a Physical AI?"
2.  **Write Lesson 2**: "The Brains of Your Robot"
3.  **Write Lesson 3**: "Giving Your Robot a Body"
4.  **Review and Edit**: Review the entire chapter for clarity, accuracy, and consistency.

### Phase 2: Future Chapters (to be planned)

*   This plan will be updated as more chapters are defined in the specification.

### Phase 3: Chapter 6 - Vision-Language-Action (VLA)

1.  **Create Chapter Directory**: Create a new directory `docs/chapter-6-vision-language-action`.
2.  **Create Category File**: Inside the new directory, create a `_category_.json` file with the label "Chapter 6: Vision-Language-Action (VLA)" and the appropriate position.
3.  **Write VLA Concepts Lesson**: Create a markdown file `1-vla-concepts.md` inside the chapter directory.
    *   **Content**: Detail the Vision-Language-Action (VLA) model, explaining the synergy between vision, language, and action.
    *   **Content**: Explain the Voice-to-Action pipeline using OpenAI's Whisper to translate speech to text within a ROS 2 framework.
    *   **Content**: Describe the process of using an LLM as a cognitive planner to break down high-level commands into a sequence of executable robot actions.
    *   **Content**: Outline the capstone project that integrates all concepts, demonstrating an autonomous humanoid executing a voice command.
4.  **Review and Edit**: Review the chapter for technical accuracy, clarity, and adherence to the book's tone.

## 3. File Structure for Chapters and Lessons

The file structure will follow the guidelines in the specification document.

### Example:

```
docs/
├── intro.md
└── chapter-1-introduction/
    ├── _category_.json
    ├── 1-what-is-a-physical-ai.md
    ├── 2-the-brains-of-your-robot.md
    └── 3-giving-your-robot-a-body.md
```

### `_category_.json` file:

```json
{
  "label": "Chapter 1: Introduction",
  "position": 2
}
```

This structure ensures a clean, organized, and scalable foundation for the book.
