# Developer Setup & Build Guide - IMU Motion Tracer

This guide provides step-by-step developer instructions for setting up, building, debugging, and extending the **IMU Motion Tracer** project, complete with actual configuration code snippets.

---

## 1. Prerequisites & Environment Setup

### Required Tools
1. **Android Studio**: Jellyfish (2023.3.1) or newer recommended.
2. **Java Development Kit (JDK)**: JDK 17 (embedded Android Studio JBR: `C:\Program Files\Android\Android Studio\jbr`).
3. **Android NDK**: Version `25.x` or newer (installed via SDK Manager: `Tools > SDK Manager > SDK Tools > NDK (Side-by-side)`).
4. **CMake**: Version `3.22.1` or newer (installed via SDK Manager: `Tools > SDK Manager > SDK Tools > CMake`).

---

## 2. Compiling & Building the Application

### Command Line Build (Windows PowerShell)

```powershell
# Set JAVA_HOME to Android Studio's bundled JDK
$env:JAVA_HOME="C:\Program Files\Android\Android Studio\jbr"

# Clean build artifacts
.\gradlew.bat clean

# Compile Debug APK & Native Shared Library (.so)
.\gradlew.bat assembleDebug
```

Output APK path:  
`app/build/outputs/apk/debug/app-debug.apk`

---

## 3. Native C++ CMake Configuration

Native compilation is managed by [`CMakeLists.txt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/CMakeLists.txt).

#### 💻 Actual `CMakeLists.txt` Code
```cmake
cmake_minimum_required(VERSION 3.22.1)
project("motion-engine")

add_library(
    motion-engine
    SHARED
    motion-engine.cpp
)

find_library(
    log-lib
    log
)

target_link_libraries(
    motion-engine
    ${log-lib}
)
```

- Target shared library name: `libmotion-engine.so`
- Linked system libraries: Android NDK Log library (`liblog.so`).

---

## 4. App Gradle Build Configuration

The app build links CMake via `externalNativeBuild`:

#### 💻 Actual `app/build.gradle.kts` Code
```kotlin
android {
    namespace = "com.example.imumotiontracer"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.example.imumotiontracer"
        minSdk = 26
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        externalNativeBuild {
            cmake {
                cppFlags("")
            }
        }
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }
}
```

---

## 5. JNI Memory Debugging & Logcat Inspecting

Native logging uses `__android_log_print` with tag `MotionEngine`.

```powershell
# View real-time native logs via adb
adb logcat -s "MotionEngine:D"
```

Sample output:
```text
D MotionEngine: Navigine parameters synced: K=0.520, threshold=0.490, filterAlpha=0.150
D MotionEngine: Step #1: Weinberg Stride=0.74m, Heading=45.2°, Position=(0.52, 0.52)
D MotionEngine: Step #2: Weinberg Stride=0.76m, Heading=45.0°, Position=(1.06, 1.06)
```

---

## 6. Extending the Codebase

### Adding a New JNI Method
1. Declare method in [`motion-engine.h`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.h).
2. Implement C++ method in [`motion-engine.cpp`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp). Ensure `std::lock_guard<std::mutex> lock(engineMutex)` is acquired.
3. Export JNI C-function in `extern "C"` block following name convention:
   `Java_com_example_imumotiontracer_NativeMotionEngine_<methodName>`
4. Add external function declaration and Kotlin wrapper in [`NativeMotionEngine.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt).
