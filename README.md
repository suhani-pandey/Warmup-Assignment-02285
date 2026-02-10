# AI Search Client - Hospital Domain (02285 Warmup Assignment)

## 📋 Project Overview

This is a Python-based AI search client for solving pathfinding and planning problems in the Hospital Domain. The client implements various search algorithms (BFS, DFS, A*, Greedy) to control robots navigating a grid-based hospital environment, moving boxes to designated locations.

**Course:** 02285 AI and Multi-Agent Systems  
**Assignment:** Warmup Assignment  
**Due Date:** 16 February at 20:00  
**Team Size:** 5 members

---

## 🚀 Setup Instructions

### Prerequisites

- **Python 3.7+** ([Download here](https://www.python.org/downloads/))
- **Java Runtime (JRE)** for running the server ([Download here](https://www.java.com/download/))
- **Git** for version control ([Download here](https://git-scm.com/downloads))

### Step 1: Download Course Materials

1. Download `searchclient.zip` from the course website
2. Extract the archive to a folder (e.g., `~/ai-project/`)
3. You should have:
   ```
   ~/ai-project/
   ├── server.jar
   ├── levels/
   ├── searchclient_python/
   ```

### Step 2: Clone This Repository

```bash
# Navigate to your project folder
cd ~/ai-project/

# Clone this repository 
git clone https://github.com/YOUR-USERNAME/YOUR-REPO-NAME.git searchclient

# Your final structure should be:
# ~/ai-project/
# ├── server.jar          (from course download)
# ├── levels/             (from course download)
# └── searchclient/       (this GitHub repo)
```

### Step 3: Install Python Dependencies

```bash
# Navigate to the searchclient folder
cd searchclient

# Install required packages
pip install psutil

# OR install from pyproject.toml
pip install -e .
```

### Step 4: Verify Setup

```bash
# Test that Java server works
java -jar ../server.jar -h

# Should display help information about server options

# Test client connection (from searchclient/ folder)
java -jar ../server.jar -l ../levels/SAD1.lvl -c "python -m searchclient.searchclient" -g -s 150

# A GUI window should open showing the level
# The client will execute some moves (currently hardcoded, won't solve the level)
```

✅ **If you see the GUI and the client connects, setup is complete!**

---

## 📁 Project Structure

```
searchclient/
├── searchclient/              # Main Python package
│   ├── __init__.py           # Package initializer
│   ├── searchclient.py       # Main entry point & server communication
│   ├── graphsearch.py        # Graph-Search algorithm (YOU IMPLEMENT)
│   ├── frontier.py           # Frontier data structures (YOU IMPLEMENT)
│   ├── heuristic.py          # Heuristic functions (YOU IMPLEMENT)
│   ├── state.py              # State representation (YOU EXTEND)
│   ├── action.py             # Action definitions (YOU EXTEND)
│   ├── color.py              # Color definitions (PROVIDED)
│   └── memory.py             # Memory monitoring (PROVIDED)
├── pyproject.toml            # Python project configuration
├── readme-searchclient.txt   # Original course readme
├── .gitignore                # Git ignore rules
└── README.md                 # This file
```

---




**Last Updated:** [Date]  
**Repository:** [Your GitHub URL]
