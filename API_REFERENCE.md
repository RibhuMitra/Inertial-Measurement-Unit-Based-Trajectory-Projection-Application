# API Reference - IMU Motion Tracer

This document provides a detailed API specification for all public classes, native C++ methods, JNI interfaces, and data models in the **IMU Motion Tracer** codebase, including actual code blocks for every declaration.

---

## Table of Contents
1. [Kotlin JNI Wrapper (`NativeMotionEngine`)](#1-kotlin-jni-wrapper-nativemotionengine)
2. [Native C++ Core (`MotionEngine`)](#2-native-c-core-motionengine)
3. [Native JNI C-Bindings](#3-native-jni-c-bindings)
4. [Wi-Fi Fingerprinting & Data Layer](#4-wi-fi-fingerprinting--data-layer)
5. [Jetpack Compose UI & State Models](#5-jetpack-compose-ui--state-models)

---

## 1. Kotlin JNI Wrapper (`NativeMotionEngine`)

Location: [`NativeMotionEngine.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L7-L139)

The `NativeMotionEngine` class manages the native pointer lifecycle and provides thread-safe accessors to the C++ motion backend.

### 💻 Class & Constructor Definition
From [`NativeMotionEngine.kt:L7-L20`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L7-L20):
```kotlin
class NativeMotionEngine(
    stepLength: Float = 0.75f,
    sensitivity: Float = 1.2f,
    filterAlpha: Float = 0.15f
) {
    // Stores the memory pointer to the native C++ MotionEngine instance
    private var nativePtr: Long = 0

    init {
        // Load the shared library compiled by CMake/NDK
        System.loadLibrary("motion-engine")
        // Instantiate the native C++ object and store its pointer
        nativePtr = createEngine(stepLength, sensitivity, filterAlpha)
    }
```

---

### Public Methods & Actual Code

#### `setParameters(stepLength: Float, sensitivity: Float, filterAlpha: Float)`
From [`NativeMotionEngine.kt:L25-L29`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L25-L29):
```kotlin
fun setParameters(stepLength: Float, sensitivity: Float, filterAlpha: Float) {
    if (nativePtr != 0L) {
        setParameters(nativePtr, stepLength, sensitivity, filterAlpha)
    }
}
```

#### `processSensorData(x: Float, y: Float, z: Float, timestampNs: Long): Boolean`
From [`NativeMotionEngine.kt:L35-L41`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L35-L41):
```kotlin
fun processSensorData(x: Float, y: Float, z: Float, timestampNs: Long): Boolean {
    return if (nativePtr != 0L) {
        processSensorData(nativePtr, x, y, z, timestampNs)
    } else {
        false
    }
}
```

#### `addStep(headingDegrees: Float, isSimulator: Boolean)`
From [`NativeMotionEngine.kt:L46-L50`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L46-L50):
```kotlin
fun addStep(headingDegrees: Float, isSimulator: Boolean) {
    if (nativePtr != 0L) {
        addStep(nativePtr, headingDegrees, isSimulator)
    }
}
```

#### `updateHeading(headingDegrees: Float)`
From [`NativeMotionEngine.kt:L55-L59`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L55-L59):
```kotlin
fun updateHeading(headingDegrees: Float) {
    if (nativePtr != 0L) {
        updateHeading(nativePtr, headingDegrees)
    }
}
```

#### `fuseLocation(wifiX: Float, wifiY: Float, beta: Float)`
From [`NativeMotionEngine.kt:L64-L68`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L64-L68):
```kotlin
fun fuseLocation(wifiX: Float, wifiY: Float, beta: Float) {
    if (nativePtr != 0L) {
        fuseLocation(nativePtr, wifiX, wifiY, beta)
    }
}
```

#### `reset()`
From [`NativeMotionEngine.kt:L73-L77`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L73-L77):
```kotlin
fun reset() {
    if (nativePtr != 0L) {
        resetEngine(nativePtr)
    }
}
```

#### `getPathPoints(): FloatArray`
From [`NativeMotionEngine.kt:L105-L108`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L105-L108):
```kotlin
fun getPathPoints(): FloatArray {
    if (nativePtr == 0L) return floatArrayOf(0.0f, 0.0f)
    return getPathPoints(nativePtr) ?: floatArrayOf(0.0f, 0.0f)
}
```

#### `destroy()`
From [`NativeMotionEngine.kt:L113-L118`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L113-L118):
```kotlin
fun destroy() {
    if (nativePtr != 0L) {
        destroyEngine(nativePtr)
        nativePtr = 0
    }
}
```

#### External JNI Methods
From [`NativeMotionEngine.kt:L128-L138`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/NativeMotionEngine.kt#L128-L138):
```kotlin
private external fun createEngine(stepLength: Float, sensitivity: Float, filterAlpha: Float): Long
private external fun destroyEngine(nativePtr: Long)
private external fun setParameters(nativePtr: Long, stepLength: Float, sensitivity: Float, filterAlpha: Float)
private external fun processSensorData(nativePtr: Long, x: Float, y: Float, z: Float, timestampNs: Long): Boolean
private external fun updateHeading(nativePtr: Long, headingDegrees: Float)
private external fun addStep(nativePtr: Long, headingDegrees: Float, isSimulator: Boolean)
private external fun fuseLocation(nativePtr: Long, wifiX: Float, wifiY: Float, beta: Float)
private external fun resetEngine(nativePtr: Long)
private external fun getStepCount(nativePtr: Long): Int
private external fun getDistance(nativePtr: Long): Float
private external fun getPathPoints(nativePtr: Long): FloatArray?
```

---

## 2. Native C++ Core (`MotionEngine`)

Header: [`motion-engine.h`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.h#L23-L95)

### 💻 Native Data Structures
From [`motion-engine.h:L8-L21`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.h#L8-L21):
```cpp
struct Point {
    float x;
    float y;
};

struct SensorSample {
    float magnitude;
    long long timestampMs;
};

struct FilteredSample {
    float value;
    long long timestampMs;
};
```

### 💻 C++ Class Declaration
From [`motion-engine.h:L23-L95`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.h#L23-L95):
```cpp
class MotionEngine {
private:
    mutable std::mutex engineMutex;

    float weinbergK;
    float minVariation;
    float filterAlpha;

    std::deque<SensorSample> accelMeasurements;
    std::deque<FilteredSample> filteredAccMagnitudes;
    std::deque<long long> stepTimes;

    bool isStep;
    long long lastStepTimeMs;
    long long prevStepTimeMs;
    float nextStride;

    float currentX;
    float currentY;
    float currentHeading;
    
    float smoothedSin;
    float smoothedCos;

    std::vector<Point> pathPoints;
    int stepCount;
    float totalDistance;

public:
    MotionEngine(float weinbergK = 0.52f, float minVariation = 0.49f, float filterAlpha = 0.15f);
    ~MotionEngine() = default;

    void setParameters(float weinbergK, float minVariation, float filterAlpha);
    bool processSensorData(float x, float y, float z, long long timestampNs);
    void updateHeading(float headingDegrees);
    void addStep(float headingDegrees, bool isSimulator);
    void fuseLocation(float wifiX, float wifiY, float beta);
    void reset();

    int getStepCount() const;
    float getDistance() const;
    float getX() const;
    float getY() const;
    std::vector<float> getPathPoints() const;
};
```

---

## 3. Native JNI C-Bindings

Declared in [`motion-engine.cpp:L283-L388`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L283-L388):

#### 💻 Actual C-Export Code
```cpp
extern "C" {

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

JNIEXPORT jboolean JNICALL
Java_com_example_imumotiontracer_NativeMotionEngine_processSensorData(
        JNIEnv* env, jobject thiz, jlong native_ptr, jfloat x, jfloat y, jfloat z, jlong timestamp_ns) {
    auto* engine = reinterpret_cast<MotionEngine*>(native_ptr);
    if (engine != nullptr) {
        return engine->processSensorData(x, y, z, timestamp_ns) ? JNI_TRUE : JNI_FALSE;
    }
    return JNI_FALSE;
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_imumotiontracer_NativeMotionEngine_getPathPoints(
        JNIEnv* env, jobject thiz, jlong native_ptr) {
    auto* engine = reinterpret_cast<MotionEngine*>(native_ptr);
    if (engine == nullptr) return nullptr;

    std::vector<float> points = engine->getPathPoints();
    jfloatArray result = env->NewFloatArray(points.size());
    if (result != nullptr) {
        env->SetFloatArrayRegion(result, 0, points.size(), points.data());
    }
    return result;
}

} // extern "C"
```

---

## 4. Wi-Fi Fingerprinting & Data Layer

### `WifiKNNLocalizer`
Location: [`WifiKNNLocalizer.kt`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/data/WifiKNNLocalizer.kt#L5-L15)

#### 💻 Data Model: `Fingerprint`
```kotlin
data class Fingerprint(
    val x: Float, // grid column index (0.0 to 7.0)
    val y: Float, // grid row index (0.0 to 7.0)
    val rssiVector: FloatArray
)
```

---

## 5. Jetpack Compose UI & State Models

### `MainScreenViewModel`
Location: [`MainScreenViewModel.kt:L13-L27`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/ui/main/MainScreenViewModel.kt#L13-L27)

#### 💻 ViewModel & UI State Code
```kotlin
class MainScreenViewModel(dataRepository: DataRepository) : ViewModel() {
  val uiState: StateFlow<MainScreenUiState> =
    dataRepository.data
      .map<List<String>, MainScreenUiState>(::Success)
      .catch { emit(MainScreenUiState.Error(it)) }
      .stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), MainScreenUiState.Loading)
}

sealed interface MainScreenUiState {
  object Loading : MainScreenUiState

  data class Error(val throwable: Throwable) : MainScreenUiState

  data class Success(val data: List<String>) : MainScreenUiState
}
```
