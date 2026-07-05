#include "AirSimDataCollector4.h"
#include "Async/Async.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "HAL/FileManager.h"
#include <atomic>
#include <memory>
#include <fstream>
#include <chrono>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include "AirLib/include/vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "AirLib/include/common/ImageCaptureBase.hpp"
#include "AirLib/include/sensors/imu/ImuBase.hpp"
#include "AirLib/include/sensors/barometer/BarometerBase.hpp"
#include "AirLib/include/sensors/magnetometer/MagnetometerBase.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

using namespace msr::airlib;

AAirSimDataCollector4::AAirSimDataCollector4()
{
    PrimaryActorTick.bCanEverTick = false;
}

void AAirSimDataCollector4::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Warning, TEXT("AirSimDataCollector4::BeginPlay()"));
    UE_LOG(LogTemp, Warning, TEXT("Camera Configuration: %dx%d @ %.1f Hz"), ImageWidth, ImageHeight, CameraHz);
    UE_LOG(LogTemp, Warning, TEXT("IMU Frequency: %.1f Hz"), ImuHz);

    if (bUseKinematicsOverride) {
        UE_LOG(LogTemp, Warning, TEXT("=== HIGH-SPEED KINEMATICS MODE ENABLED ==="));
        UE_LOG(LogTemp, Warning, TEXT("Target Speed: %.1f m/s (%.1f km/h)"),
            TargetSpeed, TargetSpeed * 3.6f); // | Acceleration: %.1f m/s\u00b2 , Acceleration
        UE_LOG(LogTemp, Warning, TEXT("This bypasses aerodynamic limits - ideal for DM-VIO datasets"));
    }

    SessionDir = MakeSessionDir();
    ImageDir = FPaths::Combine(SessionDir, TEXT("images/"));

    UE_LOG(LogTemp, Warning, TEXT("Session Directory: %s"), *SessionDir);
    UE_LOG(LogTemp, Warning, TEXT("Image Directory: %s"), *ImageDir);

    if (!EnsureDir(SessionDir) || !EnsureDir(ImageDir)) {
        UE_LOG(LogTemp, Error, TEXT("CRITICAL: Failed to create output directories."));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("Directories created. Opening files..."));
    OpenFiles();

    UE_LOG(LogTemp, Warning, TEXT("Creating calibration files..."));
    CreateCalibrationFiles();
    CreateCameraImuConfigFile();
    CreateImuSettingsFile();
    UE_LOG(LogTemp, Warning, TEXT("All configuration files created successfully!"));

    // starting writer threads
    bRunWriter.store(true);
    bRunImageWriter.store(true);
    WriterThread = std::thread([this]() { WriterWorker(); });
    ImageWriterThread = std::thread([this]() { ImageWriterWorker(); });

    UE_LOG(LogTemp, Warning, TEXT("Writer threads started. Starting connection thread..."));

    // the connection thread
    ConnectionThread = std::thread([this]() {
        try {
            const char* ip = TCHAR_TO_UTF8(*RpcAddress);
            const int port = RpcPort > 0 ? RpcPort : 41451;
            UE_LOG(LogTemp, Log, TEXT("Connecting to AirSim at %s:%d"), *RpcAddress, port);

            UE_LOG(LogTemp, Log, TEXT("Creating AirSim RPC client for STEP-BASED simulation..."));
            AirSimClient = std::make_unique<MultirotorRpcLibClient>(ip, port, 5.0f);
            AirSimClient->confirmConnection();

            AirSimClient->enableApiControl(true); // enabling API control and disable real-time mode

            // parsing the initial time of day from TimeOfDayString
            if (bEnableTimeOfDay) {
                try {
                    // parsing "2026-02-15 13:00:00" to extract the hour (13.0)
                    std::string timeStr = std::string(TCHAR_TO_UTF8(*TimeOfDayString));
                    size_t hourPos = timeStr.find_last_of(' ');
                    if (hourPos != std::string::npos && timeStr.length() > hourPos + 8) {
                        std::string timeComponent = timeStr.substr(hourPos + 1); // "13:00:00"
                        int hour = 0, minute = 0, second = 0;
                        sscanf(timeComponent.c_str(), "%d:%d:%d", &hour, &minute, &second);
                        CurrentHour = static_cast<float>(hour) + (minute / 60.0f) + (second / 3600.0f);
                        UE_LOG(LogTemp, Warning, TEXT("Parsed starting time: %.2f hours (from %s)"), CurrentHour, *TimeOfDayString);
                    } else {
                        CurrentHour = 13.0f; // default to 1 PM
                        UE_LOG(LogTemp, Warning, TEXT("Could not parse TimeOfDayString, defaulting to 13:00"));
                    }

                    // setting the initial time of day using our loop method
                    NextTimeUpdateAt = 0.0;  // update immediately
                    SetTimeOfDay();
                    UE_LOG(LogTemp, Warning, TEXT("Loop-based time control enabled: SunSpeed=%.2f hours/real-second, UpdateInterval=%.2fs"), 
                        SunSpeed, TimeUpdateInterval);
                }
                catch (const std::exception& e) {
                    UE_LOG(LogTemp, Warning, TEXT("Time of day control not available: %s"), *FString(e.what()));
                }
            }

            // initializing weather to clear conditions
            AirSimClient->simEnableWeather(true);
            SetWeatherParameters(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            if (bEnableWeatherCycle) {
                UE_LOG(LogTemp, Warning, TEXT("Weather cycle enabled: Fog → Rain → Snow → Leaves"));
            }

            UE_LOG(LogTemp, Warning, TEXT("AirSim connected! Starting STEP-BASED simulation..."));
            UE_LOG(LogTemp, Warning, TEXT("PERFECT TIMING MODE: IMU @ %.1f Hz"), ImuHz);
            UE_LOG(LogTemp, Warning, TEXT("Output directory: %s"), *SessionDir);

            // starting step-based simulation thread
            bRunSimulation.store(true);
            SimulationThread = std::thread([this]() { SimulationWorker(); });

            UE_LOG(LogTemp, Warning, TEXT("Step-based simulation active"));
        }
        catch (const std::exception& e) {
            UE_LOG(LogTemp, Error, TEXT("AirSim connection failed: %s"), *FString(e.what()));
            bRunWriter.store(false);
            bRunImageWriter.store(false);
            {
                std::lock_guard<std::mutex> lock(LogQueueMutex);
                LogQueue.push(FLogItem3{ ELogKind3::STOP });
            }
            LogQueueCv.notify_one();
            {
                std::lock_guard<std::mutex> lock(ImageQueueMutex);
                FImageItem3 stop_item;
                stop_item.IsStop = true;
                ImageQueue.push(std::move(stop_item));
            }
            ImageQueueCv.notify_one();
        }
        });

    UE_LOG(LogTemp, Warning, TEXT("AirSimDataCollector4 initialized!"));
}

void AAirSimDataCollector4::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    UE_LOG(LogTemp, Warning, TEXT("=========================================================="));
    UE_LOG(LogTemp, Warning, TEXT("AirSimDataCollector4 shutdown starting..."));
    UE_LOG(LogTemp, Warning, TEXT("=========================================================="));

    // stopping all threads
    bShuttingDown.store(true);
    bRunSimulation.store(false);
    bRunWriter.store(false);
    bRunImageWriter.store(false);

    UE_LOG(LogTemp, Warning, TEXT("Stopping step-based simulation..."));

    // sending STOP signals to queues
    {
        std::lock_guard<std::mutex> lock(LogQueueMutex);
        LogQueue.push(FLogItem3{ ELogKind3::STOP });
    }
    LogQueueCv.notify_all();

    {
        std::lock_guard<std::mutex> lock(ImageQueueMutex);
        FImageItem3 stop_item;
        stop_item.IsStop = true;
        ImageQueue.push(std::move(stop_item));
    }
    ImageQueueCv.notify_all();

    UE_LOG(LogTemp, Warning, TEXT("Waiting for threads to complete..."));

    // joining threads with generous timeouts
    if (ConnectionThread.joinable()) {
        JoinThreadWithTimeout(ConnectionThread, TEXT("Connection"), 2);
    }

    JoinThreadWithTimeout(SimulationThread, TEXT("Simulation"), 2);
    JoinThreadWithTimeout(WriterThread, TEXT("Writer"), 3);
    JoinThreadWithTimeout(ImageWriterThread, TEXT("ImageWriter"), 3);

    UE_LOG(LogTemp, Warning, TEXT("All threads stopped. Closing files..."));

    CloseFiles();

    UE_LOG(LogTemp, Warning, TEXT("Files closed. Releasing AirSim client..."));

    if (AirSimClient) {
        msr::airlib::MultirotorRpcLibClient* LeakedClient = AirSimClient.release();
        (void)LeakedClient; // suppressing unused variable warning
        UE_LOG(LogTemp, Warning, TEXT("AirSimClient pointer released (intentional leak to prevent deadlock)"));
    }

    UE_LOG(LogTemp, Warning, TEXT("=========================================================="));
    UE_LOG(LogTemp, Warning, TEXT("AirSimDataCollector4 shutdown completed successfully!"));
    UE_LOG(LogTemp, Warning, TEXT("=========================================================="));
    Super::EndPlay(EndPlayReason);
}

// ============================================================================
// STEP-BASED SIMULATION WORKER - Perfect Timing for All Sensors
// ============================================================================
void AAirSimDataCollector4::SimulationWorker()
{
    UE_LOG(LogTemp, Warning, TEXT("STEP-BASED SIMULATION"));
    UE_LOG(LogTemp, Warning, TEXT("Target Frequencies: IMU=%.1f Hz, Camera=%.1f Hz, GPS=%.1f Hz"),
        ImuHz, CameraHz, GpsHz);

    // calculating sample intervals (in seconds)
    const double imu_interval = 1.0 / std::max(1.0, static_cast<double>(ImuHz));
    const double camera_interval = 1.0 / std::max(1.0, static_cast<double>(CameraHz));
    const double gps_interval = 1.0 / std::max(1.0, static_cast<double>(GpsHz));
    const double gt_interval = 1.0 / std::max(1.0, static_cast<double>(GroundTruthHz));
    const double baro_interval = 1.0 / std::max(1.0, static_cast<double>(BarometerHz));
    const double mag_interval = 1.0 / std::max(1.0, static_cast<double>(MagnetometerHz));

    // next capture times for each sensor
    double next_imu_time = 0.0;
    double next_camera_time = 0.0;
    double next_gps_time = 0.0;
    double next_gt_time = 0.0;
    double next_baro_time = 0.0;
    double next_mag_time = 0.0;

    uint64_t imu_count = 0;
    uint64_t camera_count = 0;
    uint64_t gps_count = 0;

    SimulationTime = 0.0;

    // Initialize Kinematic State (for God Mode high-speed control)
    CurrentSpeed = 0.0f;
    CurrentPosX = 0.0;
    CurrentPosY = 0.0;
    CurrentPosZ = -400.0; // -20.0;  // Start 20 meters above ground
    StartPosX = CurrentPosX;
    StartPosY = CurrentPosY;
    StartPosZ = CurrentPosZ;
    bMovementCompleted = false;

    if (bUseKinematicsOverride) {
        // ===================================================================
        // SIZE-BASED MOVEMENT DURATION CALCULATION
        // ===================================================================
        // Goal: Drone moves at EXACTLY TargetSpeed through a pattern of size MovementSize
        // Formula: Total Distance = Pattern Size × Pattern Perimeter Factor
        //          Duration = Total Distance / Target Speed

        // pattern-specific perimeter calculations:
        const double SQUARE_PERIMETER_FACTOR = 4.0;  // Square has 4 equal sides
        const double FIGURE8_PERIMETER_FACTOR = 6.283185307179586;  // 2π (figure-8 arc length approximation)

        // Determine perimeter factor based on pattern type
        double perimeterFactor;
        const TCHAR* patternName;
        if (DroneMovementPattern == 2) {
            // SQUARE PATTERN:
            // Perimeter = 4 × SideLength
            // Total distance = 4 × MovementSize
            perimeterFactor = SQUARE_PERIMETER_FACTOR;
            patternName = TEXT("Square");
        }
        else {
            // FIGURE-8 PATTERN:
            // Arc length ≈ 2π × Amplitude (for lemniscate curve)
            // Total distance = 2π × MovementSize
            perimeterFactor = FIGURE8_PERIMETER_FACTOR;
            patternName = TEXT("Figure-8");
        }

        const double totalDistance = MovementSize * perimeterFactor; // calculating total path length from pattern size

        // calculating required duration to achieve target speed (Duration = Distance / Speed)
        if (TargetSpeed <= 0.0f) {
            UE_LOG(LogTemp, Error, TEXT("ERROR: TargetSpeed must be positive! Current: %.2f m/s"), TargetSpeed);
            TargetSpeed = 1.0f;  // Fallback to prevent division by zero
        }

        CalculatedMovementDuration = totalDistance / TargetSpeed;

        // updating the UPROPERTY for display in editor (read-only)
        MovementDuration = static_cast<float>(CalculatedMovementDuration);

        UE_LOG(LogTemp, Warning, TEXT("========================================================"));
        UE_LOG(LogTemp, Warning, TEXT("       SIZE-BASED KINEMATIC MOVEMENT (GOD MODE)         "));
        UE_LOG(LogTemp, Warning, TEXT("========================================================"));
        UE_LOG(LogTemp, Warning, TEXT("Pattern Type:        %s"), patternName);
        UE_LOG(LogTemp, Warning, TEXT("Pattern Size:        %.2f meters (side length/amplitude)"), MovementSize);
        UE_LOG(LogTemp, Warning, TEXT("Target Speed:        %.2f m/s (%.1f km/h)"), TargetSpeed, TargetSpeed * 3.6f);
        UE_LOG(LogTemp, Warning, TEXT("Calculation:         %.2f m × %.4f = %.2f m total path"),
            MovementSize, perimeterFactor, totalDistance);
        UE_LOG(LogTemp, Warning, TEXT("                     %.2f m ÷ %.2f m/s = %.2f seconds"),
            totalDistance, TargetSpeed, CalculatedMovementDuration);
        UE_LOG(LogTemp, Warning, TEXT("Flight Duration:     %.2f seconds"), CalculatedMovementDuration);
        UE_LOG(LogTemp, Warning, TEXT("Simulation Step:     %.1f ms (%.0f Hz)"), SimulationTimeStep * 1000.0, 1.0 / SimulationTimeStep);
        UE_LOG(LogTemp, Warning, TEXT("========================================================"));
    }

    UE_LOG(LogTemp, Warning, TEXT("Simulation loop starting with %.4f second time steps"), SimulationTimeStep);

    while (bRunSimulation.load() && !bShuttingDown.load()) {
        try {
            // ===================================================================
            // KINEMATIC STATE UPDATE (God Mode - Precise Speed Control)
            // ===================================================================
            // The drone position is updated each simulation step (1ms (instead of 10ms) intervals)
            // Position changes create implicit velocity: v = Δposition / Δtime
            // This ensures EXACT speed matching without aerodynamic simulation

            if (bUseKinematicsOverride && !bMovementCompleted) {

                // Calculate progress through the movement (0.0 to 1.0)
                double movementProgress = SimulationTime / CalculatedMovementDuration;

                if (movementProgress <= 1.0) {
                    // means still executing the pattern
                    if (DroneMovementPattern == 1) {
                        // FIGURE-8 PATTERN (Lemniscate of Bernoulli)
                        // ==========================================
                        // Parametric equations:
                        //   x(t) = A × sin(ω×t)
                        //   y(t) = (A/2) × sin(2×ω×t)
                        // Where:
                        //   A = amplitude (MovementSize)
                        //   ω = angular frequency = 2π / T
                        //   T = CalculatedMovementDuration (period)
                        // Arc length ≈ 2π × A (one complete cycle)
                        // Average speed = (2π × A) / T = TargetSpeed ✓

                        double omega = 2.0 * 3.14159265358979323846 / CalculatedMovementDuration;
                        double t = SimulationTime;

                        CurrentPosX = StartPosX + MovementSize * sin(omega * t);
                        CurrentPosY = StartPosY + (MovementSize / 2.0) * sin(2.0 * omega * t);
                        CurrentPosZ = StartPosZ;  // maintaining constant altitude
                    }
                    else if (DroneMovementPattern == 2) {
                        // SQUARE PATTERN
                        // ===============
                        // 4 sides, each of length S = MovementSize
                        // Total perimeter = 4 × S
                        // Each side takes T/4 seconds
                        // Speed on each side = S / (T/4) = 4S/T = TargetSpeed ✓

                        const double segmentDuration = CalculatedMovementDuration / 4.0;
                        const int segment = static_cast<int>(SimulationTime / segmentDuration);
                        const double segmentProgress = (SimulationTime - segment * segmentDuration) / segmentDuration;

                        // clamp to valid segment range [0, 3]
                        const int clampedSegment = (segment >= 4) ? 3 : segment;
                        const double clampedProgress = (segment >= 4) ? 1.0 : segmentProgress;


                        // updating current square segment for weather system
                        CurrentSquareSegment = clampedSegment;

                        switch (clampedSegment) {
                        case 0:  // Side 1: Move forward (positive X direction)
                            CurrentPosX = StartPosX + MovementSize * clampedProgress;
                            CurrentPosY = StartPosY;
                            break;
                        case 1:  // Side 2: Move right (positive Y direction)
                            CurrentPosX = StartPosX + MovementSize;
                            CurrentPosY = StartPosY + MovementSize * clampedProgress;
                            break;
                        case 2:  // Side 3: Move backward (negative X direction)
                            CurrentPosX = StartPosX + MovementSize * (1.0 - clampedProgress);
                            CurrentPosY = StartPosY + MovementSize;
                            break;
                        case 3:  // Side 4: Move left (negative Y direction) - return to start
                            CurrentPosX = StartPosX;
                            CurrentPosY = StartPosY + MovementSize * (1.0 - clampedProgress);
                            break;
                        }
                        CurrentPosZ = StartPosZ;  // Maintain constant altitude
                    }
                }
                else {
                    // movement completed (AND returning to exact start position)
                    CurrentPosX = StartPosX;
                    CurrentPosY = StartPosY;
                    CurrentPosZ = StartPosZ;

                    if (!bMovementCompleted) {
                        bMovementCompleted = true;
                        UE_LOG(LogTemp, Warning, TEXT("=== MOVEMENT PATTERN COMPLETED ==="));
                        UE_LOG(LogTemp, Warning, TEXT("Returned to start position (%.2f, %.2f, %.2f)"),
                            CurrentPosX, CurrentPosY, CurrentPosZ);
                        UE_LOG(LogTemp, Warning, TEXT("Capturing final sensor readings..."));

                        // capturing final readings at completion
                        uint64_t final_timestamp = static_cast<uint64_t>(SimulationTime * 1e9);
                        CaptureIMU(final_timestamp);
                        CaptureGPS(final_timestamp);
                        CaptureGroundTruth(final_timestamp);
                        CaptureImage(final_timestamp);

                        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // a small delay to ensure all data is queued

                        UE_LOG(LogTemp, Warning, TEXT("Initiating light shutdown..."));

                        // signalling all threads to stop
                        bRunSimulation.store(false);
                    }
                }

                // building the Pose (Position + Orientation)
                msr::airlib::Pose pose;
                pose.position = msr::airlib::Vector3r(CurrentPosX, CurrentPosY, CurrentPosZ);
                pose.orientation = msr::airlib::Quaternionr(1, 0, 0, 0); // flat orientation (no rotation)

                // forcing the pose into AirSim (bypasses physics engine)
                // The velocity/acceleration will be implicitly derived by AirSim from position changes
                AirSimClient->simSetVehiclePose(pose, false);  // false = respect collision

            }


            // updating weather system before advancing simulation
            if (bEnableWeatherCycle) {
                UpdateWeatherSystem();
            }

            // ===================================================================
            // TIME OF DAY UPDATE (Loop-Based Sun Control)
            // ===================================================================
            // Instead of relying on AirSim's CelestialClockSpeed, we manually
            // increment CurrentHour and call SetTimeOfDay() at regular intervals.
            // This gives us precise control over the lighting changes.
            
            if (bEnableTimeOfDay && SimulationTime >= NextTimeUpdateAt) {
                // incrementing CurrentHour based on simulation time and SunSpeed
                // SunSpeed = hours to advance per real-time second
                // Since last update: elapsed = TimeUpdateInterval
                // Hour increment = SunSpeed * TimeUpdateInterval
                CurrentHour += SunSpeed * static_cast<float>(TimeUpdateInterval);
                
                // wrapping around 24-hour clock
                while (CurrentHour >= 24.0f) {
                    CurrentHour -= 24.0f;
                }
                while (CurrentHour < 0.0f) {
                    CurrentHour += 24.0f;
                }
                
                // calling SetTimeOfDay to update the sun position
                SetTimeOfDay();
                
                // scheduling next update
                NextTimeUpdateAt = SimulationTime + TimeUpdateInterval;
            }

            // advancing simulation by one time step
            AirSimClient->simContinueForTime(SimulationTimeStep);
            SimulationTime += SimulationTimeStep;

            uint64_t timestamp_ns = static_cast<uint64_t>(SimulationTime * 1e9); // converting simulation time to nanoseconds for timestamps


            // checking and capturing each sensor at its precise interval
            if (SimulationTime >= next_imu_time) {
                CaptureIMU(timestamp_ns);
                next_imu_time += imu_interval;
                imu_count++;
            }

            if (SimulationTime >= next_camera_time) {
                CaptureImage(timestamp_ns);
                next_camera_time += camera_interval;
                camera_count++;
            }

            if (SimulationTime >= next_gps_time) {
                CaptureGPS(timestamp_ns);
                next_gps_time += gps_interval;
                gps_count++;
            }

            if (SimulationTime >= next_gt_time) {
                CaptureGroundTruth(timestamp_ns);
                next_gt_time += gt_interval;
            }

            if (SimulationTime >= next_baro_time) {
                CaptureBarometer(timestamp_ns);
                next_baro_time += baro_interval;
            }

            if (SimulationTime >= next_mag_time) {
                CaptureMagnetometer(timestamp_ns);
                next_mag_time += mag_interval;
            }


            // progress logging every 5 seconds of simulation time
            if (imu_count > 0 && imu_count % (static_cast<uint64_t>(ImuHz) * 5) == 0) {
                if (bUseKinematicsOverride) {
                    double patternProgress = (SimulationTime / MovementDuration) * 100.0;
                    if (patternProgress > 100.0) patternProgress = 100.0;

                    UE_LOG(LogTemp, Log, TEXT("Sim Time: %.2fs | Pattern: %.1f%% | Pos: (%.1f, %.1f, %.1f) | IMU: %llu | Cam: %llu | GPS: %llu"),
                        SimulationTime, patternProgress, CurrentPosX, CurrentPosY, CurrentPosZ, imu_count, camera_count, gps_count);
                }
                else {
                    UE_LOG(LogTemp, Log, TEXT("Sim Time: %.2fs | IMU: %llu | Cam: %llu | GPS: %llu"),
                        SimulationTime, imu_count, camera_count, gps_count);
                }
            }


            // checking if movement is completed and we should stop
            if (bMovementCompleted) {
                UE_LOG(LogTemp, Warning, TEXT("Movement completed - exiting simulation loop"));
                break;
            }
        }
        catch (const std::exception& e) {
            if (!bShuttingDown.load()) {
                UE_LOG(LogTemp, Error, TEXT("Simulation error: %s"), *FString(e.what()));
            }
            break;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("Step-based simulation stopped. Total samples:"));
    UE_LOG(LogTemp, Warning, TEXT("  IMU: %llu | Camera: %llu | GPS: %llu"), imu_count, camera_count, gps_count);
    UE_LOG(LogTemp, Warning, TEXT("  Simulation time: %.2f seconds"), SimulationTime);

    if (bUseKinematicsOverride) {
        const TCHAR* PatternName = (DroneMovementPattern == 1) ? TEXT("Figure-8") :
            (DroneMovementPattern == 2) ? TEXT("Square") : TEXT("Unknown");
        UE_LOG(LogTemp, Warning, TEXT("  Movement Pattern: %s (Size: %.1f m)"), PatternName, MovementSize);
        UE_LOG(LogTemp, Warning, TEXT("  Final Position: (%.2f, %.2f, %.2f)"), CurrentPosX, CurrentPosY, CurrentPosZ);
        UE_LOG(LogTemp, Warning, TEXT("  Movement Completed: %s"), bMovementCompleted ? TEXT("YES") : TEXT("NO"));
    }
}

// ============================================================================
// SENSOR CAPTURE FUNCTIONS - Called at precise intervals by simulation loop
// ============================================================================

void AAirSimDataCollector4::CaptureIMU(uint64_t timestamp_ns)
{
    try {
        const auto imu = AirSimClient->getImuData("Imu");

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(10);

        oss << timestamp_ns << " "
            << imu.angular_velocity[0] << " " << imu.angular_velocity[1] << " " << imu.angular_velocity[2] << " "
            << imu.linear_acceleration[0] << " " << imu.linear_acceleration[1] << " " << imu.linear_acceleration[2] << "\n";

        FLogItem3 item;
        item.Kind = ELogKind3::IMU;
        item.Line = oss.str();
        {
            std::lock_guard<std::mutex> lock(LogQueueMutex);
            if (LogQueue.size() < MAX_LOG_QUEUE_SIZE) {
                LogQueue.push(std::move(item));
                LogQueueCv.notify_one();
            }
        }
    }
    catch (...) {
        // silently skipping errors during shutdown
    }
}

void AAirSimDataCollector4::CaptureGPS(uint64_t timestamp_ns)
{
    try {
        const auto gps = AirSimClient->getGpsData("Gps");

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(10);
        oss << timestamp_ns << " "
            << gps.gnss.geo_point.latitude << " "
            << gps.gnss.geo_point.longitude << " "
            << gps.gnss.geo_point.altitude << "\n";

        FLogItem3 item;
        item.Kind = ELogKind3::GPS;
        item.Line = oss.str();
        {
            std::lock_guard<std::mutex> lock(LogQueueMutex);
            if (LogQueue.size() < MAX_LOG_QUEUE_SIZE) {
                LogQueue.push(std::move(item));
                LogQueueCv.notify_one();
            }
        }
    }
    catch (...) {
    }
}

void AAirSimDataCollector4::CaptureGroundTruth(uint64_t timestamp_ns)
{
    try {
        const auto state = AirSimClient->getMultirotorState();
        const auto& pose = state.kinematics_estimated.pose;

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(10);
        oss << timestamp_ns << ","
            << pose.position.x() << "," << pose.position.y() << "," << pose.position.z() << ","
            << pose.orientation.w() << "," << pose.orientation.x() << "," << pose.orientation.y() << "," << pose.orientation.z() << "\n";

        FLogItem3 item;
        item.Kind = ELogKind3::GT_IMU;
        item.Line = oss.str();
        {
            std::lock_guard<std::mutex> lock(LogQueueMutex);
            if (LogQueue.size() < MAX_LOG_QUEUE_SIZE) {
                LogQueue.push(std::move(item));
                LogQueueCv.notify_one();
            }
        }
    }
    catch (...) {
    }
}


void AAirSimDataCollector4::CaptureBarometer(uint64_t timestamp_ns)
{
    try {
        const std::vector<std::string> baro_names = { "", "Barometer", "baro", "0" };

        for (const std::string& baro_name : baro_names) {
            try {
                const auto barometer = AirSimClient->getBarometerData(baro_name);

                std::ostringstream oss;
                oss.setf(std::ios::fixed);
                oss.precision(10);
                oss << timestamp_ns << " " << barometer.altitude << "\n";

                FLogItem3 item;
                item.Kind = ELogKind3::BAROMETER;
                item.Line = oss.str();
                {
                    std::lock_guard<std::mutex> lock(LogQueueMutex);
                    if (LogQueue.size() < MAX_LOG_QUEUE_SIZE) {
                        LogQueue.push(std::move(item));
                        LogQueueCv.notify_one();
                    }
                }
                break;
            }
            catch (...) {
                continue;
            }
        }
    }
    catch (...) {
    }
}

void AAirSimDataCollector4::CaptureMagnetometer(uint64_t timestamp_ns)
{
    try {
        const std::vector<std::string> mag_names = { "", "Magnetometer", "magnetometer", "Compass", "compass", "0" };

        for (const std::string& mag_name : mag_names) {
            try {
                const auto magnetometer = AirSimClient->getMagnetometerData(mag_name);

                // calculating heading in degrees
                double heading_rad = atan2(magnetometer.magnetic_field_body.y(), magnetometer.magnetic_field_body.x());
                double heading_deg = heading_rad * 180.0 / 3.14159265358979323846;
                if (heading_deg < 0) heading_deg += 360.0;

                std::ostringstream oss;
                oss.setf(std::ios::fixed);
                oss.precision(10);
                oss << timestamp_ns << " " << heading_deg << "\n";

                FLogItem3 item;
                item.Kind = ELogKind3::MAGNETOMETER;
                item.Line = oss.str();
                {
                    std::lock_guard<std::mutex> lock(LogQueueMutex);
                    if (LogQueue.size() < MAX_LOG_QUEUE_SIZE) {
                        LogQueue.push(std::move(item));
                        LogQueueCv.notify_one();
                    }
                }
                break;
            }
            catch (...) {
                continue;
            }
        }
    }
    catch (...) {
    }
}

void AAirSimDataCollector4::CaptureImage(uint64_t timestamp_ns)
{
    try {
        std::vector<ImageCaptureBase::ImageRequest> request = {
            ImageCaptureBase::ImageRequest(
                "down",
                ImageCaptureBase::ImageType::Scene,
                false,
                true
            )
        };
        const auto responses = AirSimClient->simGetImages(request);

        if (!responses.empty() && !responses[0].image_data_uint8.empty()) {
            const auto& img = responses[0];
            std::string filename = std::to_string(timestamp_ns) + ".png";

            FImageItem3 image_item;
            image_item.ImageBytes = img.image_data_uint8;
            image_item.ImageFilename = filename;
            image_item.timestamp_ns = timestamp_ns;
            image_item.IsStop = false;

            // adding to image queue (non-blocking)
            {
                std::unique_lock<std::mutex> lock(ImageQueueMutex);
                if (ImageQueue.size() < MAX_IMAGE_QUEUE_SIZE) {
                    ImageQueue.push(std::move(image_item));
                    lock.unlock();
                    ImageQueueCv.notify_one();
                }
            }

            // writing to times.txt
            {
                std::lock_guard<std::mutex> lock(LogQueueMutex);
                if (LogQueue.size() < MAX_LOG_QUEUE_SIZE) {
                    FLogItem3 times_item;
                    times_item.Kind = ELogKind3::TIMES;

                    double timestamp_seconds = static_cast<double>(timestamp_ns) / 1000000000.0;
                    std::ostringstream times_oss;
                    times_oss.setf(std::ios::fixed);
                    times_oss.precision(9);
                    times_oss << std::to_string(timestamp_ns) << " " << timestamp_seconds << " 0.0";

                    times_item.Line = times_oss.str();
                    LogQueue.push(std::move(times_item));
                    LogQueueCv.notify_one();
                }
            }
        }
    }
    catch (...) {
    }
}

void AAirSimDataCollector4::WriterWorker()
{
    UE_LOG(LogTemp, Log, TEXT("DATASET WRITER THREAD"));

    while (bRunWriter.load()) {
        FLogItem3 item;
        {
            std::unique_lock<std::mutex> lock(LogQueueMutex);
            LogQueueCv.wait(lock, [this] { return !LogQueue.empty() || !bRunWriter.load(); });
            if (LogQueue.empty()) continue;
            item = std::move(LogQueue.front());
            LogQueue.pop();
        }

        switch (item.Kind) {
        case ELogKind3::IMU:
            if (ImuFile && ImuFile->is_open()) (*ImuFile) << item.Line;
            break;
        case ELogKind3::GPS:
            if (GpsFile && GpsFile->is_open()) (*GpsFile) << item.Line;
            break;
        case ELogKind3::GT_IMU:
            if (GtImuFile && GtImuFile->is_open()) (*GtImuFile) << item.Line;
            break;
        case ELogKind3::TIMES:
            if (TimesFile && TimesFile->is_open()) (*TimesFile) << item.Line << "\n";
            break;
        case ELogKind3::BAROMETER:
            if (BarometerFile && BarometerFile->is_open()) {
                (*BarometerFile) << item.Line;
                BarometerFile->flush();
            }
            break;
        case ELogKind3::MAGNETOMETER:
            if (MagnetometerFile && MagnetometerFile->is_open()) {
                (*MagnetometerFile) << item.Line;
                MagnetometerFile->flush();
            }
            break;
        case ELogKind3::STOP:
            UE_LOG(LogTemp, Log, TEXT("Writer thread received STOP signal"));
            return;
        }
    }
}

void AAirSimDataCollector4::ImageWriterWorker()
{
    UE_LOG(LogTemp, Log, TEXT("DATASET PNG WRITER - Nanosecond Filenames"));

    uint64_t images_written = 0;
    uint64_t write_errors = 0;

    while (bRunImageWriter.load()) {
        FImageItem3 item;
        {
            std::unique_lock<std::mutex> lock(ImageQueueMutex);

            ImageQueueCv.wait(lock, [this] {
                return !ImageQueue.empty() || !bRunImageWriter.load();
                });

            if (ImageQueue.empty() && !bRunImageWriter.load()) {
                break;
            }

            if (ImageQueue.empty()) {
                continue;
            }

            item = std::move(ImageQueue.front());
            ImageQueue.pop();
        }

        if (item.IsStop) {
            UE_LOG(LogTemp, Log, TEXT("Image writer received STOP signal"));
            break;
        }

        if (!item.ImageBytes.empty()) {
            FString ImagePath = ImageDir + FString(item.ImageFilename.c_str());

            if (WriteImageFile(ImagePath, item.ImageBytes)) {
                images_written++;

                if (images_written % 100 == 0) {
                    UE_LOG(LogTemp, Log, TEXT("PNG writer: %llu images written"), images_written);
                }
            }
            else {
                write_errors++;
            }
        }

        ImageQueueCv.notify_one();
    }

    UE_LOG(LogTemp, Log, TEXT("PNG writer stopped: %llu images written, %llu errors."),
        images_written, write_errors);
}

bool AAirSimDataCollector4::WriteImageFile(const FString& FilePath, const std::vector<uint8_t>& ImageData)
{
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

    TUniquePtr<IFileHandle> FileHandle(PlatformFile.OpenWrite(*FilePath));
    if (!FileHandle.IsValid())
    {
        return false;
    }

    bool bSuccess = FileHandle->Write(ImageData.data(), ImageData.size());
    FileHandle->Flush();

    return bSuccess;
}

FString AAirSimDataCollector4::MakeSessionDir() const
{
    const FString Base = TEXT("I:/Akef/Logs");

    FDateTime Now = FDateTime::Now();
    FString Timestamp = Now.ToString(TEXT("%Y%m%d_%H%M%S"));

    return FPaths::Combine(Base, Timestamp) + TEXT("/");
}

bool AAirSimDataCollector4::EnsureDir(const FString& Path) const
{
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
    bool bSuccess = PlatformFile.CreateDirectoryTree(*Path);

    if (bSuccess) {
        UE_LOG(LogTemp, Log, TEXT("Created directory: %s"), *Path);
    }
    else {
        if (PlatformFile.DirectoryExists(*Path)) {
            return true;
        }
        UE_LOG(LogTemp, Error, TEXT("Failed to create directory: %s"), *Path);
    }

    return bSuccess;
}

void AAirSimDataCollector4::OpenFiles()
{
    // IMU
    ImuFile = std::make_unique<std::ofstream>(TCHAR_TO_UTF8(*(SessionDir + TEXT("imu.txt"))), std::ios::out);
    if (ImuFile && ImuFile->is_open()) {
        (*ImuFile) << "# timestamp[ns] w.x w.y w.z a.x a.y a.z\n";
    }

    // GPS
    GpsFile = std::make_unique<std::ofstream>(TCHAR_TO_UTF8(*(SessionDir + TEXT("gps.txt"))), std::ios::out);
    if (GpsFile && GpsFile->is_open()) {
        (*GpsFile) << "# timestamp[ns] lat long alt\n";
    }

    // GT_IMU
    GtImuFile = std::make_unique<std::ofstream>(TCHAR_TO_UTF8(*(SessionDir + TEXT("gt_imu.txt"))), std::ios::out);
    if (GtImuFile && GtImuFile->is_open()) {
        (*GtImuFile) << "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz\n";
    }

    // TIMES
    TimesFile = std::make_unique<std::ofstream>(TCHAR_TO_UTF8(*(SessionDir + TEXT("times.txt"))), std::ios::out);
    if (TimesFile && TimesFile->is_open()) {
        (*TimesFile) << "# filename          timestamp[s]         [exposuretime[ms]]\n";
    }

    // BAROMETER
    BarometerFile = std::make_unique<std::ofstream>(TCHAR_TO_UTF8(*(SessionDir + TEXT("barometer.txt"))), std::ios::out);
    if (BarometerFile && BarometerFile->is_open()) {
        (*BarometerFile) << "# timestamp[ns] altitude\n";
        BarometerFile->flush();
    }

    // MAGNETOMETER
    MagnetometerFile = std::make_unique<std::ofstream>(TCHAR_TO_UTF8(*(SessionDir + TEXT("magnetometer.txt"))), std::ios::out);
    if (MagnetometerFile && MagnetometerFile->is_open()) {
        (*MagnetometerFile) << "# timestamp[ns] heading_degrees\n";
        MagnetometerFile->flush();
    }

    UE_LOG(LogTemp, Warning, TEXT("All dataset files opened successfully"));
}

void AAirSimDataCollector4::CloseFiles()
{
    if (ImuFile && ImuFile->is_open()) ImuFile->close();
    if (GpsFile && GpsFile->is_open()) GpsFile->close();
    if (GtImuFile && GtImuFile->is_open()) GtImuFile->close();
    if (TimesFile && TimesFile->is_open()) TimesFile->close();
    if (BarometerFile && BarometerFile->is_open()) BarometerFile->close();
    if (MagnetometerFile && MagnetometerFile->is_open()) MagnetometerFile->close();
}

void AAirSimDataCollector4::CreateCalibrationFiles()
{
    FString CameraCalibFile = SessionDir + TEXT("calib.txt");
    std::ofstream CalibFile(TCHAR_TO_UTF8(*CameraCalibFile), std::ios::out);
    if (CalibFile.is_open()) {
        float width = static_cast<float>(ImageWidth);
        float height = static_cast<float>(ImageHeight);
        float fov_degrees = 90.0f;

        float fov_rad = fov_degrees * 3.14159265f / 180.0f;
        float focal_length = (width / 2.0f) / tan(fov_rad / 2.0f);

        float fx = focal_length;
        float fy = focal_length;
        float cx = width / 2.0f;
        float cy = height / 2.0f;

        float k1 = 0.0f;
        float k2 = 0.0f;
        float p1 = 0.0f;
        float p2 = 0.0f;

        CalibFile << "RadTan " << fx << " " << fy << " " << cx << " " << cy << " "
            << k1 << " " << k2 << " " << p1 << " " << p2 << "\n";
        CalibFile << ImageWidth << " " << ImageHeight << "\n";
        CalibFile << "crop\n";
        CalibFile << ImageWidth << " " << ImageHeight << "\n";
        CalibFile.close();
        UE_LOG(LogTemp, Log, TEXT("Camera calibration created: %s"), *CameraCalibFile);
    }
}

void AAirSimDataCollector4::CreateCameraImuConfigFile()
{
    FString CamchainFile = SessionDir + TEXT("camchain.yaml");
    std::ofstream ConfigFile(TCHAR_TO_UTF8(*CamchainFile), std::ios::out);
    if (ConfigFile.is_open()) {
        float width = static_cast<float>(ImageWidth);
        float height = static_cast<float>(ImageHeight);
        float fov_degrees = 90.0f;
        float fov_rad = fov_degrees * 3.14159265f / 180.0f;
        float focal_length = (width / 2.0f) / tan(fov_rad / 2.0f);

        float pitch_rad = -90.0f * 3.14159265f / 180.0f;

        ConfigFile << "cam0:\n";
        ConfigFile << "  T_cam_imu:\n";
        ConfigFile << "  - [1.0, 0.0, 0.0, 0.0]\n";
        ConfigFile << "  - [0.0, " << cos(pitch_rad) << ", " << -sin(pitch_rad) << ", 0.0]\n";
        ConfigFile << "  - [0.0, " << sin(pitch_rad) << ", " << cos(pitch_rad) << ", 0.0]\n";
        ConfigFile << "  - [0, 0, 0, 1]\n";
        ConfigFile << "  cam_overlaps: [1]\n";
        ConfigFile << "  camera_model: pinhole\n";
        ConfigFile << "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]\n";
        ConfigFile << "  distortion_model: radial-tangential\n";
        ConfigFile << "  intrinsics: [" << focal_length << ", " << focal_length << ", " << (width / 2.0f) << ", " << (height / 2.0f) << "]\n";
        ConfigFile << "  resolution: [" << ImageWidth << ", " << ImageHeight << "]\n";
        ConfigFile << "  rostopic: /down/image_raw\n";
        ConfigFile.close();
        UE_LOG(LogTemp, Log, TEXT("Camera-IMU config created: %s"), *CamchainFile);
    }
}

void AAirSimDataCollector4::CreateImuSettingsFile()
{
    FString ImuSettingsFile = SessionDir + TEXT("imu_settings.yaml");

    std::ofstream ImuSettingsFileStream(TCHAR_TO_UTF8(*ImuSettingsFile), std::ios::out);
    if (ImuSettingsFileStream.is_open()) {
        ImuSettingsFileStream << std::fixed << std::setprecision(17);

        ImuSettingsFileStream << "# IMU Settings from AirSim settings.json\n";
        ImuSettingsFileStream << "gyro_random_walk: " << GyroRandomWalk << "\n";
        ImuSettingsFileStream << "gyro_bias: " << GyroBias << "\n";
        ImuSettingsFileStream << "gyro_turn_on_bias: " << GyroTurnOnBias << "\n";
        ImuSettingsFileStream << "accel_random_walk: " << AccelRandomWalk << "\n";
        ImuSettingsFileStream << "accel_bias: " << AccelBias << "\n";
        ImuSettingsFileStream << "accel_turn_on_bias: " << AccelTurnOnBias << "\n";

        ImuSettingsFileStream << "\n# Legacy DM-VIO compatible parameters\n";
        ImuSettingsFileStream << "accelerometer_noise_density: " << AccelerometerNoiseDensity << "\n";
        ImuSettingsFileStream << "gyroscope_noise_density: " << GyroscopeNoiseDensity << "\n";
        ImuSettingsFileStream << "accelerometer_random_walk: " << AccelerometerRandomWalk << "\n";
        ImuSettingsFileStream << "gyroscope_random_walk: " << GyroscopeRandomWalk << "\n";

        ImuSettingsFileStream.close();
        UE_LOG(LogTemp, Log, TEXT("IMU settings created: %s"), *ImuSettingsFile);
    }
}

bool AAirSimDataCollector4::JoinThreadWithTimeout(std::thread& thread, const TCHAR* threadName, int timeoutSeconds)
{
    if (!thread.joinable()) return true;

    try {
        thread.join();
        UE_LOG(LogTemp, Log, TEXT("Thread %s joined successfully"), threadName);
        return true;
    }
    catch (const std::exception& e) {
        UE_LOG(LogTemp, Error, TEXT("Exception joining thread %s: %s"), threadName, *FString(e.what()));
        if (thread.joinable()) {
            thread.detach();
        }
        return false;
    }
    catch (...) {
        UE_LOG(LogTemp, Error, TEXT("Unknown exception joining thread %s"), threadName);
        if (thread.joinable()) {
            thread.detach();
        }
        return false;
    }
}

// ============================================================================
// WEATHER SYSTEM - Segment-Based Weather (One Weather Per Square Side)
// ============================================================================

void AAirSimDataCollector4::UpdateWeatherSystem()
{
    if (!AirSimClient || bShuttingDown.load()) return;
    if (!bUseKinematicsOverride || DroneMovementPattern != 2) return;  // only for square pattern

    // mapping square segment (0-3) to weather condition
    // Segment 0 (forward): Fog
    // Segment 1 (right): Rain
    // Segment 2 (backward): Snow
    // Segment 3 (left): Leaves

    EWeatherState targetState = EWeatherState::Fog;

    switch (CurrentSquareSegment) {
    case 0:
        targetState = EWeatherState::Fog;
        break;
    case 1:
        targetState = EWeatherState::Rain;
        break;
    case 2:
        targetState = EWeatherState::Snow;
        break;
    case 3:
        targetState = EWeatherState::Leaves;
        break;
    default:
        targetState = EWeatherState::Fog;
        break;
    }

    if (targetState != CurrentWeatherState) { // check if we need to transition to a new weather state
        SetWeatherParameters(0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // clearing all weather before transitioning

        CurrentWeatherState = targetState;
        WeatherStateStartTime = SimulationTime;
        CurrentWeatherIntensity = 0.0f;

        const TCHAR* weatherName = (targetState == EWeatherState::Fog) ? TEXT("FOG") :
            (targetState == EWeatherState::Rain) ? TEXT("RAIN") :
            (targetState == EWeatherState::Snow) ? TEXT("SNOW") :
            (targetState == EWeatherState::Leaves) ? TEXT("LEAVES") : TEXT("UNKNOWN");

        UE_LOG(LogTemp, Warning, TEXT("Weather transition → %s (Square Segment %d)"), weatherName, CurrentSquareSegment);
    }

    CurrentWeatherIntensity = 1.0f;

    // applying weather based on current state with appropriate intensities
    float fog = 0.0f, rain = 0.0f, snow = 0.0f, mapleLeaves = 0.0f, roadLeaves = 0.0f;

    switch (CurrentWeatherState) {
    case EWeatherState::Fog:
        fog = CurrentWeatherIntensity * 0.1f;  // 0.8 max for visibility
        break;
    case EWeatherState::Rain:
        rain = CurrentWeatherIntensity;  // Full rain
        break;
    case EWeatherState::Snow:
        snow = CurrentWeatherIntensity;  // Full snow in air
        break;
    case EWeatherState::Leaves:
        mapleLeaves = CurrentWeatherIntensity;  // Falling leaves
        roadLeaves = CurrentWeatherIntensity;   // Ground leaves
        break;
    default:
        break;
    }

    // always setting ALL weather parameters to ensure clean states
    SetWeatherParameters(fog, rain, snow, mapleLeaves, roadLeaves);
}

void AAirSimDataCollector4::SetWeatherParameters(float fog, float rain, float snow, float leaves, float roadLeaves)
{
    try {
        if (!AirSimClient || bShuttingDown.load()) {
            return;
        }

        using namespace msr::airlib;

        AirSimClient->simSetWeatherParameter(WorldSimApiBase::WeatherParameter::Fog, fog);
        AirSimClient->simSetWeatherParameter(WorldSimApiBase::WeatherParameter::Rain, rain);
        AirSimClient->simSetWeatherParameter(WorldSimApiBase::WeatherParameter::Snow, snow);
        AirSimClient->simSetWeatherParameter(WorldSimApiBase::WeatherParameter::MapleLeaf, leaves);

        // logging current active weather (only if any weather is active)
        if (fog > 0.01f || rain > 0.01f || snow > 0.01f || leaves > 0.01f) {
            FString activeWeather = TEXT("Active: ");
            if (fog > 0.01f) activeWeather += FString::Printf(TEXT("Fog=%.2f "), fog);
            if (rain > 0.01f) activeWeather += FString::Printf(TEXT("Rain=%.2f "), rain);
            if (snow > 0.01f) activeWeather += FString::Printf(TEXT("Snow=%.2f "), snow);
            if (leaves > 0.01f) activeWeather += FString::Printf(TEXT("Leaves=%.2f "), leaves);

            // only logging once per second to avoid spam
            static double lastLogTime = 0.0;
            if (SimulationTime - lastLogTime >= 1.0) {
                UE_LOG(LogTemp, Log, TEXT("Weather: %s"), *activeWeather);
                lastLogTime = SimulationTime;
            }
        }
    }
    catch (const std::exception& e) {
        if (!bShuttingDown.load()) {
            UE_LOG(LogTemp, Warning, TEXT("Weather parameter setting failed: %s"), *FString(e.what()));
        }
    }
}

void AAirSimDataCollector4::SetTimeOfDay()
{
    try {
        if (!AirSimClient || !bEnableTimeOfDay || bShuttingDown.load()) {
            return;
        }

        // converting CurrentHour to time string format
        // CurrentHour is in 24-hour format (e.g., 13.5 = 13:30:00)
        int hour = static_cast<int>(CurrentHour);
        int minute = static_cast<int>((CurrentHour - hour) * 60.0f);
        int second = static_cast<int>(((CurrentHour - hour) * 60.0f - minute) * 60.0f);
        
        // clamping values to valid ranges
        hour = (hour < 0) ? 0 : (hour > 23) ? 23 : hour;
        minute = (minute < 0) ? 0 : (minute > 59) ? 59 : minute;
        second = (second < 0) ? 0 : (second > 59) ? 59 : second;
        
        // formatting the time string - keeping the date from TimeOfDayString
        std::string dateStr = std::string(TCHAR_TO_UTF8(*TimeOfDayString));
        size_t spacePos = dateStr.find_last_of(' ');
        if (spacePos != std::string::npos) {
            dateStr = dateStr.substr(0, spacePos); // extracting "2026-02-15"
        } else {
            dateStr = "2026-02-15"; // fallback date
        }
        
        // building complete timestamp: "2026-02-15 HH:MM:SS"
        char timeBuffer[32];
        snprintf(timeBuffer, sizeof(timeBuffer), "%s %02d:%02d:%02d", dateStr.c_str(), hour, minute, second);
        std::string timeStr(timeBuffer);
        
        // calling AirSim API with CelestialClockSpeed=1.0 (we control the time ourselves)
        AirSimClient->simSetTimeOfDay(true, timeStr, true, 1.0f, 60.0f, true);
        
        // optional: log every hour change (reduce spam)
        static int lastLoggedHour = -1;
        if (hour != lastLoggedHour) {
            UE_LOG(LogTemp, Log, TEXT("Sun position updated: %02d:%02d:%02d (CurrentHour=%.2f)"), 
                hour, minute, second, CurrentHour);
            lastLoggedHour = hour;
        }
    }
    catch (const std::exception& e) {
        if (!bShuttingDown.load()) {
            UE_LOG(LogTemp, Warning, TEXT("Time of day setting failed: %s"), *FString(e.what()));
        }
    }
}

