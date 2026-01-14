
# 🤖 FRC Team 10913 — Robot Wranglers

### 2026 Competition Season | Game: REBUILT

Welcome to the official robot code repository for the **Robot Wranglers**, representing **Legacy Ranch High School**. This repository houses the Java code for our 2026 competition robot.

---

## 🏗️ Technical Foundation

This repository was initialized using the **[REV MAXSwerve Java Template (v2026.0)](https://github.com/REVrobotics/MAXSwerve-Java-Template)**.

---

## 🚀 Getting Started

### 1. Prerequisites

Before you begin, ensure you have the following installed:

* **[WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html):** Includes VS Code, Java, and the C++ toolchain.

### 2. The Git Workflow

To keep our code stable, we follow a **Feature Branch** workflow. **We don't push directly to `main`.**

1. **Clone the Repo:** `git clone https://github.com/Legacy-Ranch-High-School-Robotics/2026-Robot-10913.git`
2. **Update your local code:** `git pull origin main` (Always start with the latest version).
3. **Create a new branch:** `git checkout -b <prefix>/your-task-name` (See naming convention below).
4. **Save your work:** `git add .` and `git commit -m "Brief description of what you changed"`.
5. **Upload to GitHub:** `git push origin <your-branch-name>`.
6. **Merge your code:** Create a **Pull Request (PR)** on GitHub. A lead student or mentor will review your code. Once the **GitHub Action** turns green, it is safe to merge.

### 3. Branch Naming Convention

| Prefix | Usage | Example |
| --- | --- | --- |
| **`infra/`** | Repository setup, GitHub Actions, or Gradle updates. | `infra/enable-validation` |
| **`feat/`** | Adding new robot capabilities or subsystems. | `feat/intake-logic` |
| **`fix/`** | Squashing bugs or fixing hardware mappings. | `fix/swerve-offset` |
| **`test/`** | Tuning PIDs or experimental testing code. | `test/auto-pathing` |
| **`docs/`** | Updating the README or code comments. | `docs/wiring-diagram` |

---

## 💻 Building & Simulation

We use **WPILib Simulation** to test logic without needing the physical robot.

1. **Open Project:** Open this folder in the WPILib version of VS Code.
2. **Build Code:** Press `Ctrl+Shift+P` and type **Build Robot Code**.
3. **Run Simulation:** Select **Simulate Robot Code on Desktop**.
* *Use this to test your swerve drive logic and autonomous routines before touching the hardware.*

---

## ⚡ Deployment to Robot

> **Note:** Ensure you are connected to the Robot Radio (OpenMesh/Vivid-hosting) or tethered via USB/Ethernet.

1. Confirm the RoboRIO is powered and the DS (Driver Station) shows "Communications."
2. In VS Code, click the **WPILib Icon** (W) and select **Deploy Robot Code**.
3. Check the **RioLog** for any runtime errors.
