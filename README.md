# CyberCortex.AI.inference

<!-- markdownlint-disable first-line-h1 -->
<!-- markdownlint-disable html -->
<!-- markdownlint-disable no-duplicate-header -->

<div align="center">
  <img src="https://github.com/cybercortex-robotics/inference/blob/main/figures/cyc_logo.png?raw=true" width="60%" alt="CyberCortex.AI.inference" />
</div>
<hr>
<div align="center" style="line-height: 1;">
  <a href="https://www.cybercortex.ai/" target="_blank"><img alt="Homepage"
    src="https://github.com/cybercortex-robotics/inference/blob/main/figures/cyc_logo_badge.svg?raw=true"/>
  </a>
  <br>
  <a href="https://arxiv.org/abs/2409.01241"><b>Paper Link</b>👁️</a>
</div>

## 1. Introduction

CyberCortex.AI Inference is a real-time AI execution system designed for autonomous robots and complex automation systems. It operates directly on embedded hardware, enabling robots to process sensory data, perform decision-making, and execute actions efficiently.

At its core, CyberCortex.AI Inference structures computations into modular units called "Filters", which function like lightweight AI-driven building blocks. These Filters are organized within a DataBlock, allowing seamless communication between different components of a robotic system. This architecture supports local and distributed execution, meaning robots can process data independently or collaborate with cloud-based AI for enhanced decision-making.

### Key Features
- **Real-Time Processing**: Executes AI models and sensor fusion algorithms with minimal latency.
- **Modular Design**: Uses Filters to manage various tasks like perception, motion planning, and control.
- **Distributed Execution**: Can run locally on embedded devices or communicate with cloud-based AI (CyberCortex.AI Dojo).
- **Temporal Addressable Memory (TAM)**: Optimizes data flow between Filters, ensuring efficient handling of sensory inputs and outputs.

CyberCortex.AI Inference is built to power next-generation autonomous systems, providing the intelligence needed for robotic control, AI-driven automation, and real-time decision-making in dynamic environments. 🚀

## 2. How to Run Locally
CyberCortex.AI.inference can be deployed locally on linux, windows and android machines, using the following hardware and open-source community software:
- **Android**: Android development tools
- **Eigen**: Eigen C++ Template Library for Linear Algebra
- **ENet**: ENet Networking Library
- **FFmpeg**: Multimedia Framework for Video and Audio Processing
- **GLUT**: OpenGL Utility Toolkit
- **libconfig**: C/C++ Configuration File Processing Library
- **libcsv**: CSV File Parsing Library for C
- **cURL**: libcurl - Data Transfer Library
- **LibTorch**: PyTorch C++ API - Deep Learning Framework
- **libzip**: C Library for Handling ZIP Archives
- **MessagePack**: Efficient Binary Serialization Library
- **nlohmann/json**: JSON for Modern C++ Library
- **OctoMap**: 3D Mapping Library for Robotics
- **ONNX Runtime**: Accelerated Machine Learning Inference Engine
- **OpenCV**: Open Source Computer Vision Library
- **OpenH264**: H.264 Codec by Cisco
- **OpenSSL**: Cryptography and Secure Communication Library
- **POSIX Threads (pthreads)**: Multithreading Library for C/C++
- **Qt**: Qt Framework for GUI and Application Development (version 5.12)
- **spdlog**: Fast C++ Logging Library
- **TorchVision**: PyTorch Computer Vision Library
- **Zstandard (zstd)**: Fast Lossless Compression Algorithm - Development version

## 3. How to Compile Locally

## 4. Usage via Droids

## 5. License
This repository and its contents are proprietary and subject to a commercial license. Unauthorized copying, distribution, or modification of any part of this codebase is strictly prohibited without explicit written permission from the owner.

By accessing or using this repository, you agree to comply with the terms of the commercial license. For licensing inquiries, please contact [contact@cybercortex.ai](contact@cybercortex.ai).

## 6. Citation
```
@article{https://doi.org/10.1002/rob.22426,
  author = {Grigorescu, Sorin and Zaha, Mihai},
  title = {CyberCortex.AI: An AI-based operating system for autonomous robotics and complex automation},
  journal = {Journal of Field Robotics},
  volume = {42},
  number = {2},
  pages = {474-492},
  keywords = {artificial intelligence, autonomous navigation, autonomous robots, distributed computing, embedded artificial intelligence, embedded systems, heterogeneous robots, multi-robot systems, robot autonomy, robotics operating systems},
  doi = {https://doi.org/10.1002/rob.22426},
  url = {https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.22426},
  eprint = {https://onlinelibrary.wiley.com/doi/pdf/10.1002/rob.22426}
  year = {2025}
}
```

## 7. Contact
If you have any questions, please raise an issue or contact us at [contact@cybercortex.ai](contact@cybercortex.ai).
