# Tasks: My First AI Humanoid Book Development

**Input**: [specs/physical-ai-book/plan.md](plan.md), [specs/physical-ai-book/spec.md](spec.md)
**Prerequisites**: plan.md, spec.md

## Phase 1: Docusaurus Setup Tasks

### T1.0 Initial Docusaurus Setup
- [ ] T1.0.1 Initialize new Docusaurus project: `npx create-docusaurus@latest my-book classic`
- [ ] T1.0.2 Navigate to project directory: `cd my-book`

### T1.1 Configuration (`docusaurus.config.js`)
- [ ] T1.1.1 Update `title` in `docusaurus.config.js` to "My First AI Humanoid"
- [ ] T1.1.2 Update `tagline` in `docusaurus.config.js` to "Build your own physical AI humanoid"
- [ ] T1.1.3 Configure `url` in `docusaurus.config.js` (e.g., `https://your-username.github.io/my-book/`)
- [ ] T1.1.4 Configure `baseUrl` in `docusaurus.config.js` (e.g., `/my-book/`)
- [ ] T1.1.5 Customize `navbar` items (e.g., add link to GitHub repo, adjust home link)
- [ ] T1.1.6 Customize `footer` content (e.g., copyright, links)

### T1.2 Styling (`src/css/custom.css`)
- [ ] T1.2.1 Review and modify `src/css/custom.css` for basic branding and theme colors.

### T1.3 Sidebar Configuration
- [ ] T1.3.1 Ensure `sidebars.js` is configured to auto-generate from the `docs` directory.
- [ ] T1.3.2 Remove default Docusaurus docs in the `docs` folder that are not part of the book content. (e.g., `docs/intro.md`, `docs/tutorial-basics`, `docs/tutorial-extras`)

## Phase 2: Chapter 1 Development Tasks

### T2.0 Chapter 1 Directory Structure
- [ ] T2.0.1 Create `docs/chapter-1-introduction/` directory.
- [ ] T2.0.2 Create `docs/chapter-1-introduction/_category_.json` with:
    ```json
    {
      "label": "Chapter 1: Introduction to Physical AI",
      "position": 1
    }
    ```

### T2.1 Lesson 1: What is a Physical AI?
- [ ] T2.1.1 Create `docs/chapter-1-introduction/1-what-is-a-physical-ai.md`
- [ ] T2.1.2 Write content for Lesson 1, following the "Lesson Format" in the spec.
    - [ ] Introduction
    - [ ] Learning Objectives
    - [ ] Main Content (What is AI, What is Robotics, What is Physical AI, Examples)
    - [ ] Hands-on Project (e.g., simple thought experiment or basic concept exploration)
    - [ ] Summary
    - [ ] Next Steps

### T2.2 Lesson 2: The Brains of Your Robot
- [ ] T2.2.1 Create `docs/chapter-1-introduction/2-the-brains-of-your-robot.md`
- [ ] T2.2.2 Write content for Lesson 2, following the "Lesson Format" in the spec.
    - [ ] Introduction
    - [ ] Learning Objectives
    - [ ] Main Content (What is a Microcontroller, Arduino/Raspberry Pi overview, Basic programming concepts)
    - [ ] Hands-on Project (e.g., "Hello World" on a simulated or real microcontroller)
    - [ ] Summary
    - [ ] Next Steps

### T2.3 Lesson 3: Giving Your Robot a Body
- [ ] T2.3.1 Create `docs/chapter-1-introduction/3-giving-your-robot-a-body.md`
- [ ] T2.3.2 Write content for Lesson 3, following the "Lesson Format" in the spec.
    - [ ] Introduction
    - [ ] Learning Objectives
    - [ ] Main Content (Motors, Sensors, Power Supply, Basic Chassis Design)
    - [ ] Hands-on Project (e.g., sketching a robot design, assembling simple components)
    - [ ] Summary
    - [ ] Next Steps

### T2.4 Review and Validation
- [ ] T2.4.1 Review all Chapter 1 lessons for adherence to "Content Guidelines" (Tone, Language, Visuals, Code).
- [ ] T2.4.2 Ensure all code examples are well-documented and tested.
- [ ] T2.4.3 Verify that images and diagrams are included where necessary.
- [ ] T2.4.4 Run Docusaurus locally (`npm start`) to check rendering and navigation.

This checklist provides a structured approach to implementing the first chapter of the book. Subsequent chapters will follow a similar pattern once their content is defined in the specification.

## Phase 3: Chapter 6 Development Tasks

### T3.0 Chapter 6 Directory Structure
- [x] T3.0.1 Create `docs/chapter-6-vision-language-action/` directory.
- [x] T3.0.2 Create `docs/chapter-6-vision-language-action/_category_.json` with:
    ```json
    {
      "label": "Chapter 6: Vision-Language-Action (VLA)",
      "position": 6
    }
    ```

### T3.1 Lesson 1: Vision-Language-Action Concepts
- [x] T3.1.1 Create `docs/chapter-6-vision-language-action/1-vla-concepts.md`
- [x] T3.1.2 Write content for the VLA lesson.
    - [x] Introduction to VLA models.
    - [x] Detailed explanation of the Voice-to-Action pipeline using Whisper within ROS 2.
    - [x] Description of LLM-based cognitive planning with prompt engineering examples.
    - [x] Outline of the capstone project demonstrating an autonomous humanoid executing a voice command.

### T3.2 Review and Validation
- [x] T3.2.1 Review the Chapter 6 lesson for adherence to "Content Guidelines".
- [x] T3.2.2 Verify that the concepts are explained clearly and the examples are relevant.
- [x] T3.2.3 Run Docusaurus locally (`npm start`) to check rendering and navigation.
