# Data Model: Detailed Educational Content for Physical AI & Humanoid Robotics Textbook

## EducationalChapter
**Description**: The core content unit for the textbook, containing comprehensive educational material that meets specified requirements.

**Fields**:
- `id`: String - unique identifier for the chapter (e.g., "ros2-introduction", "gazebo-simulation")
- `title`: String - descriptive title of the chapter
- `module`: String - parent module (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- `wordCount`: Integer - number of words in the chapter (between 800-1200)
- `learningObjectives`: Array<String> - list of learning objectives at the beginning of the chapter
- `realWorldContext`: String - explanation of practical applications of the concepts
- `technicalContent`: String - main body content with technical explanations
- `codeSnippets`: Array<CodeSnippet> - collection of code examples with explanations
- `diagramDescriptions`: Array<String> - text descriptions of visual concepts
- `summary`: String - comprehensive summary of key concepts covered
- `exercises`: Array<String> - practice problems or activities for students
- `path`: String - file path in the documentation structure

**Relationships**:
- Contains multiple CodeSnippet entities
- Belongs to one TechnicalTopic module

## CodeSnippet
**Description**: Python/ROS 2 code examples with detailed explanations that demonstrate theoretical concepts.

**Fields**:
- `id`: String - unique identifier for the snippet
- `language`: String - programming language (typically "python" or "ros2")
- `code`: String - the actual code content
- `explanation`: String - detailed explanation of how the code works
- `relatedConcepts`: Array<String> - concepts that this code demonstrates
- `chapterId`: String - reference to parent EducationalChapter

**Relationships**:
- Belongs to one EducationalChapter

## TechnicalTopic
**Description**: Core subject areas that form the main modules of the textbook.

**Fields**:
- `id`: String - unique identifier for the topic (e.g., "ros2", "gazebo", "nvidia-isaac", "vla")
- `name`: String - display name of the topic
- `subTopics`: Array<String> - specific areas covered within the topic
- `chapters`: Array<EducationalChapter> - collection of chapters covering this topic
- `description`: String - overview of the topic area

**Relationships**:
- Contains multiple EducationalChapter entities

## DiagramDescription
**Description**: Text-based representation of visual concepts to support visual learning as required by the specification.

**Fields**:
- `id`: String - unique identifier for the diagram description
- `title`: String - brief title of the diagram concept
- `description`: String - detailed text description of the visual concept
- `visualElements`: Array<String> - list of key visual elements described
- `relatedConcepts`: Array<String> - concepts illustrated by this diagram
- `chapterId`: String - reference to parent EducationalChapter

**Relationships**:
- Belongs to one EducationalChapter

## Module
**Description**: High-level organization unit grouping related chapters and topics.

**Fields**:
- `id`: String - unique identifier for the module (e.g., "ros2-module", "gazebo-module")
- `name`: String - display name of the module
- `description`: String - overview of what the module covers
- `chapters`: Array<EducationalChapter> - chapters included in this module
- `order`: Integer - position in the overall curriculum sequence

**Relationships**:
- Contains multiple EducationalChapter entities