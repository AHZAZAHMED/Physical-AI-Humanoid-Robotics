# Physical AI & Humanoid Robotics Textbook

Welcome to the Physical AI & Humanoid Robotics textbook project! This comprehensive resource covers the fundamentals and advanced topics in humanoid robotics, from ROS 2 basics to cognitive planning with large language models.

## Overview

This textbook is designed to provide a complete learning path for students and professionals interested in humanoid robotics. It covers:

- **ROS 2 Fundamentals**: Nodes, topics, services, and robot description
- **Simulation**: Gazebo physics, Unity rendering, and sensor simulation
- **NVIDIA Isaac Platform**: Isaac Sim, Isaac ROS, and Visual SLAM
- **VLA & Humanoid Development**: Voice-to-action systems and cognitive planning
- **Hardware Integration**: Kinematics, control, and real-world deployment

## Features

- 🤖 Comprehensive coverage of humanoid robotics concepts
- 📚 Well-structured modules with hands-on examples
- 🧠 Integration with modern AI technologies (LLM, VSLAM, etc.)
- 🛠️ Practical implementation guides and best practices
- 📊 Assessment tools and capstone project definition

## Prerequisites

- Basic programming experience in Python
- Understanding of linear algebra and calculus
- Familiarity with Linux command line
- Basic knowledge of robotics concepts (helpful but not required)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/ahzaz/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Install Dependencies

Make sure you have Node.js (v18+) and npm installed:

```bash
# Install Docusaurus dependencies
npm install
```

### 3. Environment Setup

Copy the environment template and configure your settings:

```bash
cp .env .env.local
# Edit .env.local with your configuration
```

## Development

### Local Development

For development with both frontend and backend services running simultaneously:

```bash
# Start both frontend and backend services with one command
npm run dev

# This will start:
# - Frontend (Docusaurus) on http://localhost:3000
# - Backend (FastAPI) on http://localhost:8000
```

Alternatively, you can run services separately:

```bash
# Start only the frontend
npm start

# Or start only the backend (from project root)
npm run start:backend

# Open http://localhost:3000 to view the textbook
```

### Build for Production

```bash
# Build the static site
npm run build

# Serve the built site locally
npm run serve
```

## Project Structure

```
physical-ai-textbook/
├── docs/                   # Textbook content
│   ├── intro.md           # Introduction and learning outcomes
│   ├── module-1/          # ROS 2 Fundamentals
│   ├── module-2/          # Simulation
│   ├── module-3/          # NVIDIA Isaac Platform
│   ├── module-4/          # VLA & Humanoid Development
│   └── assessments/       # Hardware requirements and capstone project
├── src/                   # Custom React components and CSS
├── static/                # Static assets
├── docusaurus.config.js   # Docusaurus configuration
├── sidebars.js           # Navigation structure
└── package.json          # Dependencies and scripts
```

## Modules

### Module 1: ROS 2 Fundamentals
- ROS 2 Architecture and communication patterns
- Nodes, topics, services, and actions
- URDF and robot modeling

### Module 2: Simulation
- Gazebo physics simulation
- Unity high-fidelity rendering
- Sensor simulation techniques

### Module 3: NVIDIA Isaac Platform
- Isaac Sim for robotics simulation
- Isaac ROS perception packages
- Visual SLAM implementation

### Module 4: VLA & Humanoid Development
- Voice-to-action systems
- Cognitive planning with LLMs
- Humanoid kinematics and control

## Contributing

We welcome contributions to improve this textbook! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-topic`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add amazing topic'`)
5. Push to the branch (`git push origin feature/amazing-topic`)
6. Open a Pull Request

## RAG Chatbot Integration

This textbook includes a Retrieval-Augmented Generation (RAG) chatbot that can answer questions about the content. The backend implementation uses:

- **Embedding Model**: e5-base-v2 with "passage:" and "query:" prefixes
- **Vector Database**: Qdrant Cloud for storage
- **LLM**: Gemini API for response generation
- **Framework**: FastAPI backend with ChatKit integration

## Deployment

The textbook is designed for GitHub Pages deployment:

```bash
# Deploy to GitHub Pages
npm run deploy
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

If you encounter any issues or have questions about the content:

1. Check the [Issues](https://github.com/ahzaz/physical-ai-textbook/issues) page
2. Create a new issue if your problem isn't already reported
3. Use the integrated chatbot to ask questions about the textbook content

## Acknowledgments

- The Docusaurus team for the excellent documentation framework
- The ROS community for the foundational robotics concepts
- NVIDIA Isaac team for advanced simulation and perception tools
- The open-source community for various libraries and tools used in this project

---

**Happy learning!** 🚀

*Dive deep into the fascinating world of Physical AI and Humanoid Robotics.*