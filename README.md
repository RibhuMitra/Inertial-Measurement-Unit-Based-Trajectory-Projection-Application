# IMU Motion Tracer - Documentation Index

Welcome to the comprehensive documentation for **IMU Motion Tracer**, a native Android application for real-time Pedestrian Dead Reckoning (PDR), Inertial Measurement Unit (IMU) signal processing, Wi-Fi RSSI Fingerprinting localization, and dynamic vector visualization.

---

## 📚 Documentation Structure

The documentation is organized into focused Markdown modules within this `docs/` directory:

| Document | Description | Key Topics |
| :--- | :--- | :--- |
| 🏗️ **[ARCHITECTURE.md](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/docs/ARCHITECTURE.md)** | System architecture & component design | Android Sensor Layer, JNI Bridge, C++ NDK Engine, Wi-Fi KNN Localizer, Mutex Thread Safety |
| 📚 **[API_REFERENCE.md](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/docs/API_REFERENCE.md)** | Full class & method references | Kotlin classes, C++ `MotionEngine`, JNI exported C-functions, parameter structures |
| 🧮 **[ALGORITHM_SPECIFICATION.md](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/docs/ALGORITHM_SPECIFICATION.md)** | Mathematical algorithms & signal filtering | PDR, Weinberg Stride Estimation, LPF filtering, Circular Heading Smoothing, Wi-Fi RSSI KNN, Sensor Fusion |
| 🎨 **[UI_AND_VISUALIZATION.md](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/docs/UI_AND_VISUALIZATION.md)** | User Interface & graphics architecture | Jetpack Compose vector Canvas, transform matrix pan/zoom, live acceleration graph, simulator modes |
| 🛠️ **[DEVELOPER_GUIDE.md](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/docs/DEVELOPER_GUIDE.md)** | Developer setup & build instructions | Android Studio, NDK / CMake build, Gradle scripts, JNI debugging, memory management |

---

## 🔍 Code Base Quick Map

- **Native C++ Engine Header**: [`motion-engine.h`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.h#L23-L95)
- **Native C++ Implementation**: [`motion-engine.cpp`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L16-L277)
- **Native JNI Export Bindings**: [`motion-engine.cpp:L283-L388`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L283-L388)
- **Kotlin JNI Wrapper**: [`NativeMotionEngine.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L7-L139)
- **Wi-Fi KNN Localizer**: [`WifiKNNLocalizer.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/data/WifiKNNLocalizer.kt#L5-L112)
- **Wi-Fi Scan Manager**: [`WifiScanManager.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/data/WifiScanManager.kt#L25-L231)
- **Main Compose Dashboard**: [`MainScreen.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/ui/main/MainScreen.kt)
- **CMake Build Config**: [`CMakeLists.txt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/CMakeLists.txt)
