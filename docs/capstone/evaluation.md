---
id: capstone-evaluation
title: "Capstone Project: Evaluation"
sidebar_label: "Evaluation"
sidebar_position: 3
description: "Criteria and rubric for evaluating the Autonomous Humanoid Capstone Project, detailing how your project will be assessed for successful completion."
keywords: ['capstone', 'evaluation', 'rubric', 'grading', 'assessment']
prerequisites:
  - "capstone-milestones"
---

## Project Evaluation: Assessing Your Autonomous Humanoid

The Capstone Project culminates in a comprehensive evaluation of your autonomous humanoid system. This document outlines the criteria and rubric by which your project will be assessed. The evaluation focuses on your ability to integrate knowledge from across the modules, the functionality and robustness of your system, and the clarity of your presentation.

### Overall Assessment Philosophy

-   **Integration**: Emphasis on how well different components (perception, language, planning, action) are integrated into a cohesive system.
-   **Functionality**: The ability of the robot to successfully execute commanded tasks in the simulated environment.
-   **Robustness**: The system's performance under varying conditions and its ability to handle minor errors.
-   **Understanding**: Demonstrated comprehension of the underlying principles and technologies.

### Evaluation Criteria and Rubric

The project will be graded across the following categories, with points allocated as shown:

| Category                     | Weight | Excellent (5)                                                                | Good (3-4)                                                                           | Satisfactory (1-2)                                                                  | Needs Improvement (0)                                                        |
| :--------------------------- | :----- | :--------------------------------------------------------------------------- | :----------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------- | :--------------------------------------------------------------------------- |
| **1. System Architecture**   | 20%    | Clear, well-documented, modular ROS 2 architecture with efficient communication. | Functional ROS 2 architecture; some areas could be more modular or clearly documented. | Basic ROS 2 integration, but lacking modularity, documentation, or efficiency.      | No coherent ROS 2 architecture or significant integration issues.            |
| **2. Perception System**     | 20%    | Robust object detection and 3D pose estimation; handles minor occlusions/noise. | Functional object detection; pose estimation mostly accurate; some sensitivity to noise. | Basic object detection; pose estimation inconsistent or inaccurate.                 | Vision system largely non-functional or unable to detect required objects.   |
| **3. Language Interface**    | 15%    | Accurately parses diverse NL commands into executable actions; handles ambiguity. | Parses common NL commands; some limitations with complex sentences or ambiguity.     | Parses only very simple, specific commands; frequent misinterpretations.            | Unable to process natural language commands effectively.                     |
| **4. Decision-Making/Planning** | 15%    | Generates optimal action sequences; robust error handling/recovery; task feasibility. | Generates correct action sequences for most tasks; basic error handling.              | Generates action sequences for simple tasks; limited error handling.                 | Planning module non-functional or generates incorrect plans.                 |
| **5. Action Execution**      | 20%    | Smooth, precise navigation and manipulation; successfully completes tasks reliably. | Generally smooth navigation/manipulation; completes tasks with occasional issues.    | Jerky movements or frequent failures in navigation/manipulation; tasks rarely completed. | Robot unable to perform navigation or manipulation actions.                  |
| **6. Integration & Robustness** | 10%    | All modules seamlessly integrated; system handles unexpected events gracefully. | Most modules integrated; system sometimes fails on unexpected events.                | Partial integration of modules; system frequently crashes or behaves unpredictably. | Major integration failures; system unstable or non-operational.              |

### Pass/Fail Thresholds

To successfully pass the Capstone Project, your system MUST meet the following minimum thresholds:

-   **Functional Demonstration**: The robot MUST be able to successfully execute at least one natural language command that involves a 2-step pick-and-place task (e.g., "Pick up the red block and place it in the blue bin") in a clear, unambiguous setup.
-   **Integration**: All primary modules (Perception, Language, Planning, Action) MUST demonstrate basic integration and communication.
-   **Safety (Simulated)**: The robot MUST NOT exhibit erratic behavior (e.g., self-collision, flying away) that would be dangerous in a real-world scenario.
-   **Technical Merit**: The project MUST demonstrate a reasonable application of concepts from at least three of the four core modules (ROS 2, Simulation, Isaac, VLA).

Projects that do not meet these minimum pass thresholds will be considered incomplete and will require significant revisions.

### Grading Scale

-   **A (Excellent)**: All criteria met with outstanding performance; demonstrates innovation or advanced understanding beyond requirements. (4.5 - 5.0)
-   **B (Good)**: All criteria met; project is functional, robust, and well-designed. (3.5 - 4.4)
-   **C (Satisfactory)**: Meets minimum pass thresholds; functional but may have minor issues or areas for improvement. (2.5 - 3.4)
-   **D (Needs Improvement)**: Fails to meet minimum pass thresholds; significant issues in functionality or integration. (1.5 - 2.4)
-   **F (Incomplete)**: Project non-functional or critical failures; does not demonstrate understanding of core concepts. (0 - 1.4)

### Submission Requirements

Your final submission will include:
1.  **Code Repository**: Link to your GitHub repository with all project code, `README.md`, and clear instructions for setup and running.
2.  **Project Report**: A brief report (5-10 pages) summarizing your architecture, design choices, challenges faced, and a reflection on your learning.
3.  **Demonstration Video**: A short video (3-5 minutes) showcasing your robot executing key tasks.

Good luck with your capstone project! We look forward to seeing your innovative solutions.
