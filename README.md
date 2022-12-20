<div align="center">
    <img src="https://user-images.githubusercontent.com/105265012/182009126-780f5e24-849f-4e0d-ac0e-b429e5d0b3fe.png" alt="CARET logo">
</div>

[![build-and-test](https://github.com/tier4/CARET_trace/actions/workflows/build-and-test.yaml/badge.svg)](https://github.com/tier4/CARET_trace/actions/workflows/build-and-test.yaml)
[![pytest](https://github.com/tier4/CARET_analyze/actions/workflows/pytest.yaml/badge.svg)](https://github.com/tier4/CARET_analyze/actions/workflows/pytest.yaml)

# CARET

CARET (Chain-Aware ROS Evaluation Tool) is one of performance analysis tool dedicated with ROS 2 applications. It is able to measure not only callback latency and communication latency, but also path latency, in other words, chain of node or callback. As additional tracepoints are introduced by function hook, tracing resolution is improved.

<div align="center">
    <img src="https://user-images.githubusercontent.com/105265012/182009138-bb9892c6-fa66-488c-bbb2-631df170fcf7.png" alt="overview">
</div>

## Publications & presentations

If you find CARET is useful in your research, please consider citing:

- T. Kuboichi, A. Hasegawa, B. Peng, K. Miura, K. Funaoka, S. Kato, and T. Azumi, "CARET: Chain-Aware ROS 2 Evaluation Tool," _IEEE international conference on Embedded and Ubiquitous Computing (EUC)_, 2022.

- B. Peng, A. Hasegawa, and T. Azumi, "Scheduling Performance Evaluation Framework for ROS 2 Applications," _IEEE International Conference on Embedded Software and Systems (ICESS)_, 2022.

<details>
<summary>BibTeX</summary>

```bibtex
@inproceedings{CARET,
title={{CARET}: Chain-{Aware} {ROS} 2 {Evaluation Tool}},
author={Kuboichi, Takahisa and Hasegawa, Atsushi and Peng, Bo and Miura, Keita and Funaoka, Kenji and Kato, Shinpei and Azumi, Takuya},
booktitle={Proceedings of IEEE international conference on embedded and ubiquitous computing (EUC)},
year={2022}}
```

```bibtex
@inproceedings{callback_scheduling,
title={Scheduling Performance Evaluation Framework for {ROS} 2 Applications},
author={Peng, Bo and Hasegawa, Atsushi and Azumi, Takuya},
booktitle={Proceedings of IEEE International Conference on Embedded Software and Systems (ICESS)},
year={2022}}
```

</details>

Also, check out ROSCON 2022 presentation titled "Chain-Aware ROS Evaluation Tool (CARET)" ([video](https://vimeo.com/showcase/9954564/video/767150288), [slide](http://download.ros.org/downloads/roscon/2022/Chain-Aware%20ROS%20Evaluation%20Tool%20(CARET).pdf)).

## Documentation

- To learn about using CARET, refer to the [CARET document](https://tier4.github.io/CARET_doc/main/)
- To find API document, refer to [CARET analyze API document](https://tier4.github.io/CARET_analyze/latest/)

## Repository overview

- [caret](https://github.com/tier4/caret)
  - Meta-repository containing `.repos` files to construct a CARET workspace
- [CARET_trace](https://github.com/tier4/CARET_trace)
  - Define tracepoints added by function hooking
- [CARET_analyze](https://github.com/tier4/CARET_analyze)
  - Library for scripts to analyze and visualize data
- [CARET_analyze_cpp_impl](https://github.com/tier4/CARET_analyze_cpp_impl.git)
  - Efficient helper functions to analyze trace data written in C++
- [ros2caret](https://github.com/tier4/ros2caret.git)
  - CLI commands like `ros2 caret`
- [CARET_doc](https://github.com/tier4/CARET_doc)
  - Documentation
- [CARET_demos](https://github.com/tier4/CARET_demos)
  - Demo programs for CARET
- [rclcpp](https://github.com/tier4/rclcpp/tree/v0.3.0)
  - The forked `rclcpp` including CARET-dedicated tracepoints
- [ros2_tracing](https://github.com/tier4/ros2_tracing/tree/v0.3.0)
  - The forked `ros2_tracing` including definition of CARET-dedicated tracepoints
