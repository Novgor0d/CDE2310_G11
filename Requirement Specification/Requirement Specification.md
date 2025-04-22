# 🎯 IR-Heat Detection & Projectile Launcher System for TurtleBot3 Burger (Raspberry Pi 4)

## 📌 Mission Objective

Design and build an **IR (heat) detection module** with an integrated **projectile launcher** for a **TurtleBot3 Burger** running on **Raspberry Pi 4**, capable of completing the following tasks within a single mission:

---

## 🚗 Maze Navigation Requirements

1. The TurtleBot must **autonomously navigate a maze** no larger than **5m × 5m**.
2. Maze walls are under **1 meter** in height.

---

## 🔥 IR Detection Objectives

1. Within the maze, the robot must locate a **designated heat signature zone**, marked by an **IR-lamp** in an enclosed area that is **randomly placed**.
2. The TurtleBot must also identify **three "Target" objects** (biscuit tins), each with an **infrared heat signature ("Hot target")**.
3. Each hot target has a **vertical positional marker** placed **1.5m above** it.

---

## 🚀 Launching Requirements

- The robot must:
  - **Detect** the hot target.
  - **Orient** its position accordingly.
  - **Aim and fire** at the vertical positional marker.
  - Launch a **minimum of 3 ping pong balls** in the following **timing pattern**:  
    **2s → 4s → 2s** intervals between shots.

---

## 📋 Conditions & Constraints

1. All targets are placed **on the floor**.
2. There is **only one elevation zone** (non-critical) located in the **bonus zone**.
3. The projectile must be a **standard-sized ping pong ball**.
4. The **terminal height of the projectile must exceed the height of the maze walls**.
5. The **ping pong ball must not hit the wall or ceiling** of the surrounding room.

---
