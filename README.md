# 2025 무인 모빌리티 경진대회

<p align="center">
  <img src=".github/page.jpg" width="45.72%">
  <img src=".github/ajou-nice.jpg" width="34.3%">
</p>

**2025 창작 모빌리티 경진대회 [무인 모빌리티 경진 대회 부문]**

# Obstacle Avoidance

This code implements the obstacle avoidance algorithm for the **ERP42** platform.

You can change the path shapes and scoring methods depending on the situation.


## Small Obstacle Avoidance

<p align="center">
  <img src=".github/smallobs.gif" width="28.8%">
  <img src=".github/smallobs_real.gif" width="51.2%">
</p>

## Big Obstacle Avoidance

<p align="center">
  <img src=".github/bigobs.gif" width="40%">
  <img src=".github/bigobs_real2.gif" width="40%">
</p>

## Can be used for rubber cone driving.

<p align="center">
  <img src=".github/cone.gif" width="80%">
</p>


# System Pipeline
<p align="center">
  <img src=".github/architecture.png" width="80%">
</p>
### Data Coordination
* **Topic `/odom`**: Used for $(x, y)$ coordinates only.
* **Topic `/vehicle_yaw`**: Provides the vehicle's yaw angle in the map frame.

**Due to system connectivity, only $(x, y)$ from `/odom` is used, while the vehicle's yaw in the map frame is separately managed via `/vehicle_yaw`.**














