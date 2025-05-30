^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ld08_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.4 (2025-05-28)
------------------
* Deprecate ament_include_dependency usage in CMakeLists.txt
* Contributor: Hyungyu Kim

1.1.3 (2025-04-11)
------------------
* Support flexible configuration of the frame_id used when publishing the scan topic
* Reduce CPU usage of the ld08 LiDAR node by adding sleep to its main loop, related PR(https://github.com/ROBOTIS-GIT/ld08_driver/pull/23)
* Contributors: Hyungyu Kim, Xander Soldaat

1.1.2 (2025-04-02)
------------------
* Removed the unused parameter from the constructor of the SlTransform class
* Contributors: Hyungyu Kim

1.1.1 (2025-03-27)
------------------
* Added the boost and libudev-dev dependency for release
* Modified the CI and lint
* Contributors: Pyo, Hyungyu Kim

1.1.0 (2025-02-25)
------------------
* ROS 2 Humble Hawksbill supported
* Added Lint and CI
* Contributors: Hye-Jong Kim, Hyungyu Kim

1.0.0 (2022-02-04)
------------------
* First release
* Contributors: Will Son
