# Algorithm Specification - IMU Motion Tracer

This document presents the detailed mathematical formulas, Digital Signal Processing (DSP) algorithms, Pedestrian Dead Reckoning (PDR) equations, and Wi-Fi RSSI fingerprinting models used in the **IMU Motion Tracer** system, along with actual code implementation snippets.

---

## 1. Acceleration Vector Signal Processing

Raw accelerometer sensor updates $(a_x, a_y, a_z)$ arrive at 50–100Hz from Android hardware.

### 1.1 3D Acceleration Magnitude
To make step detection invariant to device spatial orientation, raw 3-axis readings are converted into a scalar 3D vector magnitude:

$$a_{\text{raw}}[n] = \sqrt{a_x[n]^2 + a_y[n]^2 + a_z[n]^2}$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L49-L53`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L49-L53):
```cpp
// 1. Calculate raw magnitude
float rawMag = std::sqrt(x * x + y * y + z * z);

// 2. Append raw measurement
accelMeasurements.push_back(SensorSample{rawMag, timestampMs});
```

---

### 1.2 Dynamic Gravity Estimation Window
Earth's gravity vector ($\approx 9.80665\text{ m/s}^2$) varies dynamically based on device orientation and user tilt. The algorithm maintains a sliding window of magnitude measurements over $T_{\text{avg}} = 2500\text{ ms}$:

$$a_{\text{gravity}}[n] = \frac{1}{N_{\text{avg}}} \sum_{i \in W_{2500\text{ms}}} a_{\text{raw}}[i]$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L58-L67`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L58-L67):
```cpp
// Average magnitude over the last 2500ms (Averaging Window)
double sumAverage = 0.0;
int countAverage = 0;
for (auto it = accelMeasurements.rbegin(); it != accelMeasurements.rend(); ++it) {
    if (lastMeasTs - it->timestampMs >= AVERAGING_TIME_INTERVAL_MS) {
        break;
    }
    sumAverage += it->magnitude;
    countAverage++;
}
```

---

### 1.3 Low-Pass Filter (LPF) Window
To suppress high-frequency hand tremors and noise while preserving step heel-strike impacts, a shorter sliding window mean filter is calculated over $T_{\text{filter}} = 200\text{ ms}$:

$$a_{\text{lpf}}[n] = \frac{1}{N_{\text{filter}}} \sum_{i \in W_{200\text{ms}}} a_{\text{raw}}[i]$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L69-L78`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L69-L78):
```cpp
// Filtered magnitude over the last 200ms (LPF Window)
double sumFiltered = 0.0;
int countFiltered = 0;
for (auto it = accelMeasurements.rbegin(); it != accelMeasurements.rend(); ++it) {
    if (lastMeasTs - it->timestampMs >= FILTER_TIME_INTERVAL_MS) {
        break;
    }
    sumFiltered += it->magnitude;
    countFiltered++;
}
```

---

### 1.4 Dynamic Zero-Alignment
The dynamic gravity magnitude $a_{\text{gravity}}$ is subtracted from the low-pass signal $a_{\text{lpf}}$ to isolate linear walking motion acceleration:

$$a_{\text{zero}}[n] = a_{\text{lpf}}[n] - a_{\text{gravity}}[n]$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L84-L91`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L84-L91):
```cpp
double averMagnitude = sumAverage / countAverage;
double filterMagnitude = sumFiltered / countFiltered;

// 4. Align magnitude relative to zero (Subtracts gravity vector dynamically)
filterMagnitude -= averMagnitude;

// 5. Store zero-aligned filtered magnitude
filteredAccMagnitudes.push_back(FilteredSample{static_cast<float>(filterMagnitude), lastMeasTs});
```

---

## 2. Dynamic Step Detection & Adaptive Debouncing

Step detection relies on identifying upward zero-aligned signal crossovers above a calibrated threshold (`minVariation`, default $0.49\text{ m/s}^2$).

### 2.1 Crossover Peak Criterion
A step candidate triggers when:

$$\left( a_{\text{zero}}[n-1] < \text{threshold} \right) \quad \text{AND} \quad \left( a_{\text{zero}}[n] > \text{threshold} \right)$$

### 2.2 Adaptive Step Debouncing
To eliminate false positives caused by accidental body movement, a dynamic step interval window is computed based on historical step pacing:

$$T_{\text{debounce}} = \max\left( T_{\text{min\_step\_ms}}, \; 0.6 \times \bar{T}_{\text{historical\_step}} \right)$$

Where $T_{\text{min\_step\_ms}} = 300\text{ ms}$ (minimum physical limit).

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L111-L138`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L111-L138):
```cpp
// Dynamic debounce based on average step interval times
long long averageStepTime = 0;
auto nSteps = std::max(static_cast<long long>(stepTimes.size()), 1LL);
if (stepTimes.size() >= MINIMAL_NUMBER_OF_STEPS) {
    long long sum = 0;
    for (long long t : stepTimes) {
        sum += t;
    }
    averageStepTime = sum / nSteps;
}
float timeBetweenSteps = std::max(1.0f * MIN_TIME_BETWEEN_STEPS_MS, 0.6f * averageStepTime);

bool stepDetected = false;
FilteredSample curAcc = filteredAccMagnitudes.back();
FilteredSample prevAcc = filteredAccMagnitudes[filteredAccMagnitudes.size() - 2];

// Crossover detection: crosses threshold (minVariation) in an upward direction
if (!isStep &&
    prevAcc.value < minVariation &&
    curAcc.value > minVariation &&
    curAcc.timestampMs - lastStepTimeMs > MIN_TIME_BETWEEN_STEPS_MS &&
    timeBetweenSteps > 0 &&
    curAcc.timestampMs - lastStepTimeMs > timeBetweenSteps) {
    
    isStep = true;
    prevStepTimeMs = lastStepTimeMs;
    lastStepTimeMs = curAcc.timestampMs;
    stepDetected = true;
    // ...
}
```

---

## 3. Weinberg Stride Length Estimation

Rather than using a fixed step length, the native C++ engine dynamically estimates stride length per step using the **Weinberg Model**.

### 3.1 Mathematical Model
The Weinberg equation relies on the fourth root of the vertical acceleration peak-to-peak amplitude during a step event window ($700\text{ ms}$):

$$\text{Stride} = K \cdot \sqrt[4]{a_{\text{max}} - a_{\text{min}}}$$

Where:
- $K$ is the Weinberg stride tuning parameter (default $K = 0.52$).
- $a_{\text{max}} = \max_{i \in W_{700\text{ms}}} \left( a_{\text{zero}}[i] \right)$
- $a_{\text{min}} = \min_{i \in W_{700\text{ms}}} \left( a_{\text{zero}}[i] \right)$

### 3.2 Anatomical Range Clamping
The estimated stride length is strictly clamped to valid human walking bounds:

$$\text{Stride}_{\text{clamped}} = \max\left( 0.3\text{m}, \; \min\left( \text{Stride}, 1.8\text{m} \right) \right)$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L140-L155`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L140-L155):
```cpp
// Weinberg Stride Length Estimation using 700ms window max/min peak amplitude
float maxMagn = -1e9f;
float minMagn = 1e9f;
for (const auto& fmsr : filteredAccMagnitudes) {
    if (lastMeasTs - fmsr.timestampMs <= UPDATE_TIME_INTERVAL_MS) {
        if (fmsr.value > maxMagn) maxMagn = fmsr.value;
        if (fmsr.value < minMagn) minMagn = fmsr.value;
    }
}

float accAmplitude = maxMagn - minMagn;
float stride = weinbergK * std::sqrt(std::sqrt(accAmplitude));

// Clamp step size to anatomical limits (0.3m to 1.8m)
stride = std::max(0.3f, std::min(stride, 1.8f));
nextStride = stride;
```

---

## 4. Circular Trigonometric Heading Smoothing

Directly smoothing heading angles in degrees $\theta \in [0^\circ, 360^\circ)$ using simple arithmetic averaging causes catastrophic wrap-around errors at the $0^\circ \leftrightarrow 360^\circ$ (North) boundary.

### 4.1 Sine/Cosine Component Decomposition & Smoothing
To solve wrap-around discontinuities, incoming azimuth angle $\theta_n$ is projected into unit circle components $(\sin\theta, \cos\theta)$:

$$S[n] = \alpha \cdot \sin(\theta_n) + (1 - \alpha) \cdot S[n-1]$$

$$C[n] = \alpha \cdot \cos(\theta_n) + (1 - \alpha) \cdot C[n-1]$$

$$\theta_{\text{rad}} = \text{atan2}(S[n], C[n])$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L173-L182`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L173-L182):
```cpp
void MotionEngine::updateHeading(float headingDegrees) {
    std::lock_guard<std::mutex> lock(engineMutex);
    float rad = headingDegrees * (M_PI / 180.0f);
    smoothedSin = filterAlpha * std::sin(rad) + (1.0f - filterAlpha) * smoothedSin;
    smoothedCos = filterAlpha * std::cos(rad) + (1.0f - filterAlpha) * smoothedCos;
    
    float smoothedHeadingRad = std::atan2(smoothedSin, smoothedCos);
    currentHeading = smoothedHeadingRad * (180.0f / M_PI);
    if (currentHeading < 0) currentHeading += 360.0f;
}
```

---

## 5. Pedestrian Dead Reckoning (PDR) Trajectory Integration

When a step event occurs at timestamp $t$, relative position coordinates $(x, y)$ update in 2D Euclidean space:

$$\Delta x = \text{Stride} \cdot \sin(\theta_{\text{smoothed}})$$

$$\Delta y = -\text{Stride} \cdot \cos(\theta_{\text{smoothed}})$$

$$x_{n+1} = x_n + \Delta x, \qquad y_{n+1} = y_n + \Delta y$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L185-L214`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L185-L214):
```cpp
void MotionEngine::addStep(float headingDegrees, bool isSimulator) {
    std::lock_guard<std::mutex> lock(engineMutex);

    float smoothedHeadingRad;
    if (isSimulator) {
        currentHeading = headingDegrees;
        float rad = headingDegrees * (M_PI / 180.0f);
        smoothedSin = std::sin(rad);
        smoothedCos = std::cos(rad);
        smoothedHeadingRad = rad;
    } else {
        smoothedHeadingRad = currentHeading * (M_PI / 180.0f);
    }

    // Compute path updates (Y goes down in screen coordinates)
    float dx = nextStride * std::sin(smoothedHeadingRad);
    float dy = -nextStride * std::cos(smoothedHeadingRad);

    currentX += dx;
    currentY += dy;
    
    pathPoints.push_back({currentX, currentY});
    stepCount++;
    totalDistance += nextStride;
}
```

---

## 6. Wi-Fi RSSI K-Nearest Neighbors (KNN) Fingerprinting

The Wi-Fi localization module estimates absolute spatial position from received signal strength indications (RSSI).

### 6.1 Database Construction & Feature Vector Sorting
Each grid location $j$ is represented by average RSSI values per unique BSSID:

#### 💻 Actual Code Implementation
From [`WifiKNNLocalizer.kt:L22-L57`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/data/WifiKNNLocalizer.kt#L22-L57):
```kotlin
fun buildDatabase(measurements: List<WifiMeasurement>) {
    if (measurements.isEmpty()) {
        featureBssids = emptyList()
        fingerprints = emptyList()
        return
    }
    
    // 1. Identify all unique BSSIDs to establish feature dimension
    featureBssids = measurements.map { it.bssid.uppercase() }.distinct().sorted()
    
    // 2. Group measurements by grid cell (row, col)
    val groupedByCell = measurements.groupBy { Pair(it.row, it.col) }
    val newFingerprints = mutableListOf<Fingerprint>()
    
    for ((coords, cellMs) in groupedByCell) {
        val (row, col) = coords
        val rssiVector = FloatArray(featureBssids.size) { -100f }
        
        val msByBssid = cellMs.groupBy { it.bssid.uppercase() }
        for (i in featureBssids.indices) {
            val bssid = featureBssids[i]
            val bssidMs = msByBssid[bssid]
            if (bssidMs != null) {
                val avgRssi = bssidMs.map { it.rssi.toFloat() }.average().toFloat()
                rssiVector[i] = avgRssi
            }
        }
        
        newFingerprints.add(Fingerprint(col.toFloat(), row.toFloat(), rssiVector))
    }
    
    fingerprints = newFingerprints
}
```

---

### 6.2 Inverse-Distance Weighted Regression ($k$-NN)
The Euclidean distance $d_j$ to each fingerprint $j$ is computed, selecting top $k=3$ neighbors:

$$w_j = \frac{1}{d_j + 0.001}, \qquad x_{\text{wifi}} = \frac{\sum w_j x_j}{\sum w_j}, \qquad y_{\text{wifi}} = \frac{\sum w_j y_j}{\sum w_j}$$

#### 💻 Actual Code Implementation
From [`WifiKNNLocalizer.kt:L63-L111`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/java/com/example/imumotiontracer/data/WifiKNNLocalizer.kt#L63-L111):
```kotlin
fun predictLocation(currentScan: List<android.net.wifi.ScanResult>): Pair<Float, Float>? {
    if (fingerprints.isEmpty() || featureBssids.isEmpty()) return null
    
    val currentVector = FloatArray(featureBssids.size) { -100f }
    val scanMap = currentScan.associateBy({ it.BSSID.uppercase() }, { it.level.toFloat() })
    
    var matchedFeatures = 0
    for (i in featureBssids.indices) {
        val bssid = featureBssids[i]
        if (scanMap.containsKey(bssid)) {
            currentVector[i] = scanMap[bssid]!!
            matchedFeatures++
        }
    }
    
    if (matchedFeatures == 0) return null
    
    // Compute Euclidean distance from live vector to each fingerprint
    val distances = fingerprints.map { fingerprint ->
        var sumSq = 0f
        for (i in featureBssids.indices) {
            val diff = currentVector[i] - fingerprint.rssiVector[i]
            sumSq += diff * diff
        }
        val dist = sqrt(sumSq)
        Pair(fingerprint, dist)
    }.sortedBy { it.second }
    
    val nearest = distances.take(k)
    if (nearest.isEmpty()) return null
    
    // Compute inverse-distance weighted average position
    var totalWeight = 0f
    var weightedX = 0f
    var weightedY = 0f
    
    for ((fingerprint, dist) in nearest) {
        val weight = 1f / (dist + 0.001f)
        weightedX += fingerprint.x * weight
        weightedY += fingerprint.y * weight
        totalWeight += weight
    }
    
    return Pair(weightedX / totalWeight, weightedY / totalWeight)
}
```

---

## 7. Sensor Fusion (PDR + Wi-Fi)

To eliminate cumulative drift in PDR step integration without suffering from Wi-Fi signal jitter, position coordinates undergo complementary fusion:

$$x_{\text{fused}} = (1 - \beta) x_{\text{PDR}} + \beta x_{\text{wifi}}, \qquad y_{\text{fused}} = (1 - \beta) y_{\text{PDR}} + \beta y_{\text{wifi}}$$

#### 💻 Actual Code Implementation
From [`motion-engine.cpp:L216-L223`](file:///c:/Users/user/Downloads/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main%20%281%29/Inertial-Measurement-Unit-Based-Trajectory-Projection-Application-main/app/src/main/cpp/motion-engine.cpp#L216-L223):
```cpp
void MotionEngine::fuseLocation(float wifiX, float wifiY, float beta) {
    std::lock_guard<std::mutex> lock(engineMutex);
    currentX = (1.0f - beta) * currentX + beta * wifiX;
    currentY = (1.0f - beta) * currentY + beta * wifiY;
    if (!pathPoints.empty()) {
        pathPoints.back() = {currentX, currentY};
    }
}
```
