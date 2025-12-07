# Generation Plan for "Teaching Physical AI & Humanoid Robotics"

## 1. What will be produced?
This project will produce the complete content for a 5-chapter university-level textbook titled "Teaching Physical AI & Humanoid Robotics." This includes:
- A comprehensive textbook specification (`spec.md`)
- A detailed 5-chapter outline with subsections, learning objectives, and planned diagrams (`outline.md`)
- Five full chapters, each adhering to the specified template and including an introduction, learning objectives, key concepts, teaching explanations, math/equations, examples, diagrams, lab modules, summaries, problem sets, and instructor notes.
- All ASCII diagrams as separate files or integrated into the chapters as specified.
- Six dedicated lab modules, covering basic kinematics, robot perception, control systems, learning-based control, whole-body control, and simulation experiments, each with detailed instructions, code templates, and rubrics.
- Supplementary materials: a full glossary of terms, an alphabetical index, a list of references and recommended reading, and a summary for instructors.
- The assembled final textbook in a clean, publication-ready structure.

## 2. Chapter Order of Generation
The chapters will be generated sequentially to ensure logical flow and consistent referencing.
1. Chapter 1: Introduction to Physical AI & Humanoid Robotics
2. Chapter 2: Robot Perception & Sensors
3. Chapter 3: Actuation & Control Systems
4. Chapter 4: Humanoid Kinematics & Dynamics
5. Chapter 5: Learning-Based Control for Humanoids

## 3. Naming Conventions
- **Specifications:** `spec.md`
- **Outline:** `outline.md`
- **Generation Plan:** `plan.md`
- **Chapters:** Each chapter will be saved as `chapter_N.md` (e.g., `chapter_1.md`, `chapter_2.md`).
- **Diagrams:** Diagrams will be integrated directly into the chapter markdown files, or if separate, named `diagram_C_N.md` (e.g., `diagram_1_1.md`).
- **Lab Modules:** Lab modules will be integrated directly into the chapter markdown files.
- **Supplementary Materials:**
    - Glossary: `glossary.md`
    - Index: `index.md`
    - References: `references.md`
    - Instructor Summary: `instructor_summary.md`
- **Final Assembled Book:** `textbook.md` (or similar for final render)

## 4. Dependencies
- **`spec.md`:** Must be completed and approved before `outline.md` and chapter generation begins. It defines all core requirements.
- **`outline.md`:** Must be completed and approved based on `spec.md` before individual chapter generation begins. It provides the structural blueprint for the entire book.
- **Chapters:** Each chapter depends on `spec.md` for formatting and content requirements, and `outline.md` for its specific structure and subsections. Chapters will be generated in order (1-5), with subsequent chapters potentially referencing earlier ones.
- **Diagrams and Lab Modules:** These elements are integrated within their respective chapters and depend on the chapter content for context.
- **Supplementary Materials:** These will be generated after all chapters are complete, drawing information (terms, references) from the full chapter content.
- **Final Assembly:** The `textbook.md` (or final rendered version) depends on all individual chapters and supplementary materials being fully generated and reviewed.

**Workflow:**
1.  **Generate `spec.md`** (Completed).
2.  **Generate `outline.md`** (Completed).
3.  **Generate `plan.md`** (In Progress).
4.  **Generate `chapter_1.md`** (and its integrated diagrams/labs).
5.  **Generate `chapter_2.md`** (and its integrated diagrams/labs).
6.  **Generate `chapter_3.md`** (and its integrated diagrams/labs).
7.  **Generate `chapter_4.md`** (and its integrated diagrams/labs).
8.  **Generate `chapter_5.md`** (and its integrated diagrams/labs).
9.  **Generate supplementary materials** (`glossary.md`, `index.md`, `references.md`, `instructor_summary.md`).
10. **Assemble `textbook.md`** (or final rendered version).
11. **Prepare for export and final rendering.**
