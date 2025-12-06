# Physical AI & Humanoid Robotics Textbook

Interactive textbook built with Docusaurus covering ROS 2, simulation (Gazebo/Unity), NVIDIA Isaac, and Vision-Language-Action systems for autonomous humanoid robots.

## 🚀 Quick Start

### Prerequisites

- **Node.js**: v18.0 or later
- **npm**: v9.0 or later
- **Git**: For version control

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/robotics-textbook.git
cd robotics-textbook

# Install dependencies
npm install

# Start development server
npm run start
```

The site will open at `http://localhost:3000`.

### Build

```bash
# Build for production
npm run build

# Test production build locally
npm run serve
```

## 📚 Content Structure

```
docs/
├── intro.md                      # Homepage
├── module-1-ros2/                # Module 1: ROS 2
│   ├── index.md
│   ├── 1-1-ros2-intro.md
│   ├── 1-2-nodes-topics.md
│   ├── 1-3-services-actions.md
│   ├── 1-4-parameters.md
│   ├── 1-5-urdf.md
│   └── exercises/
├── module-2-simulation/          # Module 2: Gazebo & Unity
├── module-3-isaac/               # Module 3: NVIDIA Isaac
├── module-4-vla/                 # Module 4: Vision-Language-Action
├── capstone/                     # Capstone Project
└── resources/                    # Appendices
    ├── glossary.md
    ├── hardware-specs.md
    ├── software-setup.md
    └── further-reading.md
```

## 🎯 Features

- ✅ **Interactive Code Examples**: Copy-to-clipboard functionality
- ✅ **Full-Text Search**: Local search (works offline)
- ✅ **Light/Dark Modes**: Theme toggle
- ✅ **Math Rendering**: KaTeX for equations
- ✅ **Mermaid Diagrams**: Architecture visualizations
- ✅ **Code Tabs**: Compare Python/C++ implementations
- ✅ **Accessibility**: WCAG 2.1 AA compliant
- ✅ **Responsive**: Mobile-friendly design

## 🛠️ Development

### Project Structure

```
├── docs/                 # MDX content files
├── src/
│   ├── css/             # Custom styles
│   └── components/      # React components
├── static/              # Static assets
│   └── img/            # Images and diagrams
├── docusaurus.config.js # Site configuration
├── sidebars.js          # Navigation structure
└── package.json         # Dependencies
```

### Adding Content

1. Create MDX file in appropriate `docs/` subdirectory
2. Add frontmatter (id, title, description, keywords)
3. Write content using Markdown/MDX syntax
4. Update `sidebars.js` if adding new pages
5. Test locally with `npm run start`

See `specs/001-robotics-textbook-mdx/quickstart.md` for detailed guide.

## 📖 Learning Path

### Beginner (Module 1)
Start here if new to robotics:
- [Module 1: ROS 2](./docs/module-1-ros2/index.md)

### Intermediate (Modules 2-4)
After completing Module 1:
- [Module 2: Simulation](./docs/module-2-simulation/index.md)
- [Module 3: NVIDIA Isaac](./docs/module-3-isaac/index.md)
- [Module 4: VLA](./docs/module-4-vla/index.md)

### Advanced (Capstone)
Integrate all concepts:
- [Capstone Project](./docs/capstone/index.md)

## 🧪 Testing

```bash
# Validate build
npm run build

# Check links
npx markdown-link-check docs/**/*.md

# Accessibility audit
npm run build && npx @lhci/cli autorun
```

## 🚢 Deployment

### GitHub Pages

```bash
# Build and deploy
npm run build
npm run deploy
```

### Vercel

1. Import repository in Vercel dashboard
2. Framework: Docusaurus
3. Build command: `npm run build`
4. Output directory: `build`

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/new-chapter`)
3. Add content following style guidelines
4. Test build locally
5. Submit pull request

See `specs/001-robotics-textbook-mdx/contracts/content-structure.md` for content standards.

## 📝 License

[Specify license - e.g., MIT, Creative Commons BY-SA 4.0]

## 🙏 Acknowledgments

- ROS 2 community
- Docusaurus team
- Educational robotics community

## 📧 Contact

- Issues: [GitHub Issues](https://github.com/your-org/robotics-textbook/issues)
- Discussions: [GitHub Discussions](https://github.com/your-org/robotics-textbook/discussions)

---

**Built with ❤️ using [Docusaurus](https://docusaurus.io/)**
