# 🤖 Blender to Universal Robots (UR) Live Sync Control Interface

An open-source framework for controlling **Universal Robot (UR3e)** directly from **Blender** in real-time. 

This project was developed to explore the **robotic reproduction of Traditional Chinese Ink Painting**, utilizing Motion Capture data from artist **Prof. Koon**. It features a **Geometry Nodes-based control system** that allows for parametric scaling, smoothing, and safety bounding.

---

## 📂 Files in this Repository

*   **`ur_control_script.py`**: The main Python script to run in Blender.
*   **`Instruction_Manual.pdf`**: Step-by-step guide on how to set up the IP and Network and run the script.
*   **`UR_Ink_Painting_Demo.blend`**: (Optional) The Blender project file with Geometry Nodes setup.

---

## ✨ Key Features

*   **Real-time Synchronization:** Low-latency control via TCP/IP Socket communication (Port 30003).
*   **Safety Bounding Box:** Automatically returns the robot to a "Home Position" if the target object leaves the defined safety zone (bounding box).
*   **Geometry Nodes:** Process raw motion capture data before sending it to the robot.
*   **User-Friendly UI:** A custom Blender sidebar panel to control IP, Speed, and Smoothing.

---

## 🚀 Quick Start

1.  Download the **[Instruction Manual (PDF)](Instruction_Manual.pdf)** for network setup.
2.  Connect your UR Robot via Ethernet.
3.  Open Blender and load the `ur_control_script.py`.
4.  Run the script and use the **UR Control** panel in the sidebar (N-key).

---

## 🎓 Research Background

This tool was developed as part of a research project on **"Digital Preservation of Intangible Cultural Heritage"**. 

*   **Technical Supervision:** Prof. Peter AC Nelson
*   **Artistic Data Source:** Prof. Koon (Traditional Chinese Ink Painting)

---

## 📄 License

This project is open-source. Feel free to use and modify.
