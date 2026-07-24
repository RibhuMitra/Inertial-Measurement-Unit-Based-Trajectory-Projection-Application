# System Architecture - IMU Motion Tracer

This document describes the high-level software architecture, module interactions, threading boundaries, and data lifecycles of the **IMU Motion Tracer** application, alongside actual code implementations.

---

## 1. High-Level Component Architecture

The application adopts a three-tier architecture separating hardware sensor capture, native high-performance signal processing, and reactive UI rendering.

```mermaid
graph TD
    subgraph Hardware [Android Hardware & Sensors]
        Accel["TYPE_ACCELEROMETER (x, y, z @ ~50Hz)"]
        RotVec["TYPE_ROTATION_VECTOR (Quaternions)"]
        WifiHw["Wi-Fi Chipset (Scan Results & RSSI)"]
    end

    subgraph AppLayer [Kotlin Jetpack Compose Layer]
        Activity["MainActivity.kt"]
        ComposeUI["MainScreen.kt (Canvas & Dashboard)"]
        VM["MainScreenViewModel.kt"]
        ScanManager["WifiScanManager.kt (StateFlow)"]
        Localizer["WifiKNNLocalizer.kt (k-NN Model)"]
        JNIBridge["NativeMotionEngine.kt Wrapper"]
    end

    subgraph NativeLayer [Native C++ NDK Engine]
        JNIExports["extern 'C' JNI Export Functions"]
        Engine["MotionEngine Class (C++)"]
        Mutex["std::mutex engineMutex"]
        Buffers["std::deque History Windows"]
    end

    Accel -->|SensorEventListener| ComposeUI
    RotVec -->|SensorEventListener| ComposeUI
    WifiHw -->|BroadcastReceiver| ScanManager
    
    ScanManager -->|Live ScanResults| Localizer
    Localizer -->|Estimated Coordinates (x, y)| ComposeUI

    ComposeUI -->|processSensorData / updateHeading| JNIBridge
    ComposeUI -->|fuseLocation(wifiX, wifiY)| JNIBridge

    JNIBridge -->|JNI Calls with nativePtr| JNIExports
    JNIExports -->|Lock & Delegate| Engine
    Engine --- Mutex
    Engine --- Buffers
    Engine -.->|Interleaved pathPoints FloatArray| JNIBridge
    JNIBridge -.->|State updates| ComposeUI
```

---

## 2. Architectural Layers & Actual Code Implementations

### A. Android Sensor & Hardware Ingestion
From [`MainScreen.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/ui/main/MainScreen.kt):

```kotlin
// Sensor Event Listener setup in Jetpack Compose
val sensorEventListener = remember {
    object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) return
            when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    val x = event.values[0]
                    val y = event.values[1]
                    val z = event.values[2]
                    val stepDetected = nativeEngine.processSensorData(x, y, z, event.timestamp)
                    if (stepDetected) {
                        nativeEngine.addStep(rawHeading, isSimulator = false)
                    }
                }
                Sensor.TYPE_ROTATION_VECTOR -> {
                    val orientation = FloatArray(3)
                    val rotationMatrix = FloatArray(9)
                    SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
                    SensorManager.getOrientation(rotationMatrix, orientation)
                    val azimuthDeg = (Math.toDegrees(orientation[0].toDouble()).toFloat() + 360f) % 360f
                    rawHeading = azimuthDeg
                    nativeEngine.updateHeading(azimuthDeg)
                }
            }
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }
}
```

---

### B. Kotlin JNI Wrapper Layer
From [`NativeMotionEngine.kt:L15-L21`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L15-L21):

```kotlin
init {
    // Load the shared library compiled by CMake/NDK
    System.loadLibrary("motion-engine")
    // Instantiate the native C++ object and store its pointer
    nativePtr = createEngine(stepLength, sensitivity, filterAlpha)
}
```

---

### C. Native C++ NDK Core Engine & Mutex Synchronization
From [`motion-engine.cpp:L44-L46`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L44-L46):

```cpp
bool MotionEngine::processSensorData(float x, float y, float z, long long timestampNs) {
    // Thread safety lock for all member mutations
    std::lock_guard<std::mutex> lock(engineMutex);
    
    long long timestampMs = timestampNs / 1000000LL;
    float rawMag = std::sqrt(x * x + y * y + z * z);
    accelMeasurements.push_back(SensorSample{rawMag, timestampMs});
    // ...
}
```

---

## 3. Threading Model & Synchronization

```
     [Sensor Thread / Main UI Thread]              [JNI Mutex Boundary (C++)]
                    │                                          │
 1. SensorEvent (Accel @ 50Hz)                                 │
    └─> processSensorData(x, y, z) ────────> JNI ────────> lock_guard(engineMutex)
                                                               │  ├── Calculate Magnitude
                                                               │  ├── Low-Pass Filtering
                                                               │  └── Crossover Detection
    <────────────────── returns boolean ────────────────────── unlock_guard
                    │                                          │
 2. SensorEvent (Rotation @ 50Hz)                              │
    └─> updateHeading(heading) ────────────> JNI ────────> lock_guard(engineMutex)
                                                               │  └── Circular Smoothing
    <────────────────── returns void ───────────────────────── unlock_guard
                    │                                          │
 3. Jetpack Compose Canvas Draw                                │
    └─> getPathPoints() ───────────────────> JNI ────────> lock_guard(engineMutex)
                                                               │  └── Copy points vector
    <────────────────── returns FloatArray ─────────────────── unlock_guard
```

#### 💻 C++ Mutex Implementation Block
From [`motion-engine.cpp:L268-L277`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L268-L277):
```cpp
std::vector<float> MotionEngine::getPathPoints() const {
    std::lock_guard<std::mutex> lock(engineMutex);
    std::vector<float> points;
    points.reserve(pathPoints.size() * 2);
    for (const auto& pt : pathPoints) {
        points.push_back(pt.x);
        points.push_back(pt.y);
    }
    return points;
}
```

---

## 4. Native Memory Lifecycle Management

### 💻 Heap Allocation & Deallocation Code
From [`motion-engine.cpp:L285-L297`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L285-L297):

```cpp
JNIEXPORT jlong JNICALL
Java_com_example_imumotiontracer_NativeMotionEngine_createEngine(
        JNIEnv* env, jobject thiz, jfloat step_length, jfloat sensitivity, jfloat filter_alpha) {
    auto* engine = new MotionEngine(step_length, sensitivity, filter_alpha);
    return reinterpret_cast<jlong>(engine);
}

JNIEXPORT void JNICALL
Java_com_example_imumotiontracer_NativeMotionEngine_destroyEngine(
        JNIEnv* env, jobject thiz, jlong native_ptr) {
    auto* engine = reinterpret_cast<MotionEngine*>(native_ptr);
    delete engine;
}
```

And in Kotlin [`NativeMotionEngine.kt:L113-L122`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L113-L122):

```kotlin
fun destroy() {
    if (nativePtr != 0L) {
        destroyEngine(nativePtr)
        nativePtr = 0
    }
}

protected fun finalize() {
    destroy()
}
```
