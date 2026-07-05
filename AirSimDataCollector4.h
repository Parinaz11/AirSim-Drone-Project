// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "AirLib/include/vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "AirLib/include/common/ImageCaptureBase.hpp"
#include "HAL/PlatformFilemanager.h"
#include "AirSimDataCollector4.generated.h"

enum class ELogKind3 : uint8
{
    IMU,
    GPS,
    GT_IMU,
    TIMES,
    BAROMETER,
    MAGNETOMETER,
    STOP
};

struct FLogItem3
{
    ELogKind3 Kind = ELogKind3::STOP;
    std::string Line;

    FLogItem3() = default;
    FLogItem3(ELogKind3 InKind) : Kind(InKind) {}
};

struct FImageItem3
{
    std::vector<uint8_t> ImageBytes;
    std::string ImageFilename;
    uint64_t timestamp_ns = 0;
    bool IsStop = false;

    FImageItem3() = default;
};

UCLASS()
class MYTESTPROJECT_API AAirSimDataCollector4 : public AActor
{
    GENERATED_BODY()

public:
    AAirSimDataCollector4();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // configuration properties
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AirSim Connection")
    FString RpcAddress = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AirSim Connection")
    int32 RpcPort = 41451;


    // frequencies
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor Frequencies")
    float ImuHz = 200.0f;  // for 200Hz IMU frequency

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor Frequencies")
    float GpsHz = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor Frequencies")
    float GroundTruthHz = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor Frequencies")
    float CameraHz = 10.0f; // 500.0f; // 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor Frequencies")
    float BarometerHz = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor Frequencies")
    float MagnetometerHz = 50.0f;

    // UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Settings")
    // FString CameraName = TEXT("0"); // We use "down" instead


    // camera calibration parameters
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    FString CameraModel = TEXT("Pinhole");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    float Fx = 320.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    float Fy = 320.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    float Cx = 320.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    float Cy = 240.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    int32 ImageWidth = 512;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Calibration")
    int32 ImageHeight = 512;


    // IMU Noise Parameters from settings.json
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings")
    float GyroRandomWalk = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings")
    float GyroBias = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings")
    float GyroTurnOnBias = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings")
    float AccelRandomWalk = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings")
    float AccelBias = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings")
    float AccelTurnOnBias = 0.0f;


    // keeping legacy parameters for backward compatibility
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings (Legacy)")
    float AccelerometerNoiseDensity = 0.00912504793531317f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings (Legacy)")
    float GyroscopeNoiseDensity = 0.0019425812268625707f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings (Legacy)")
    float AccelerometerRandomWalk = 0.0001305951086175891f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Settings (Legacy)")
    float GyroscopeRandomWalk = 3.9570963198826466e-05f;


    // High-Speed Kinematics Settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics Control")
    float TargetSpeed = 250.0f; // 100.0f; // 250.0f;  // Target velocity in m/s

    // UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics Control")
    // float Acceleration = 25.0f;  // Acceleration rate in m/s  (if us equal to 25.0f, then reaches 250 m/s in 10 seconds)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics Control")
    bool bUseKinematicsOverride = true;  // Enable/disable "God Mode"


    // Drone Movement Pattern Settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement Pattern")
    int32 DroneMovementPattern = 2;  // 1 = Figure-8, 2 = Square

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement Pattern")
    float MovementSize = 50.0f; // 600.0f; // 50.0f;  // size of the movement pattern in meters (side length for square, amplitude for figure-8)

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement Pattern")
    float MovementDuration = 0.0f;  // [CALCULATED] Duration to complete pattern - computed from TargetSpeed and MovementSize


    // Weather Control Settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather Control")
    bool bEnableWeatherCycle = true;  // enable/disable automatic weather transitions

    // Time of Day Control
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time of Day")
    FString TimeOfDayString = TEXT("2026-02-15 13:00:00");  // Starting time (format: YYYY-MM-DD HH:MM:SS)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time of Day")
    float SunSpeed = 0.1f;  // How many hours to advance per real-time second (0.1 = 6 min/sec, 1.0 = 1 hour/sec, 10.0 = time-lapse)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time of Day")
    float TimeUpdateInterval = 0.1f;  // How often to update sun position in simulation seconds (0.1 = every 100ms)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time of Day")
    bool bEnableTimeOfDay = true;  // enabling time of day control

private:
    // threads - Step-based simulation model
    std::thread ConnectionThread;
    std::thread SimulationThread;  // Master thread that drives simulation time
    std::thread WriterThread;
    std::thread ImageWriterThread;

    // atomic state flags
    std::atomic<bool> bRunSimulation{ false };
    std::atomic<bool> bRunWriter{ false };
    std::atomic<bool> bRunImageWriter{ false };
    std::atomic<bool> bShuttingDown{ false };


    // Step-based simulation control
    double SimulationTimeStep = 0.001; // 0.01;  // 10ms simulation steps (100Hz)
    double SimulationTime = 0.0;  // current simulation time in seconds
    std::mutex SimTimeMutex;


    // Kinematic State Trackers (for God Mode)
    float CurrentSpeed = 0.0f;
    double CurrentPosX = 0.0;
    double CurrentPosY = 0.0;
    double CurrentPosZ = -400.0; // -20.0; // -10.0;  // Start altitude (10m above ground)
    double StartPosX = 0.0;  // Starting position for return
    double StartPosY = 0.0;
    double StartPosZ = -400.0; // -20.0; // -10.0;
    bool bMovementCompleted = false;  // track when movement is done
    double CalculatedMovementDuration = 0.0;  // calculated from TargetSpeed and MovementSize


    // Weather State Tracking
    enum class EWeatherState : uint8
    {
        Fog,
        Rain,
        Snow,
        Leaves,
        Complete
    };
    EWeatherState CurrentWeatherState = EWeatherState::Fog;
    double WeatherStateStartTime = 0.0;
    double CurrentWeatherIntensity = 0.0;  // 0.0 to 1.0 for smooth transitions
    int CurrentSquareSegment = 0;  // for tracking which side of square we're on (0-3)

    // Time of Day State Tracking (for loop-based sun control)
    float CurrentHour = 13.0f;  // Current hour in 24-hour format (13.5 = 13:30)
    double NextTimeUpdateAt = 0.0;  // Next simulation time to update sun position


    // RPC Client - Single client for step-based mode (no threading conflicts)
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> AirSimClient;

    // logging queues and synchronization
    std::queue<FLogItem3> LogQueue;
    std::mutex LogQueueMutex;
    std::condition_variable LogQueueCv;

    std::queue<FImageItem3> ImageQueue;
    std::mutex ImageQueueMutex;
    std::condition_variable ImageQueueCv;

    static constexpr size_t MAX_LOG_QUEUE_SIZE = 10000;
    static constexpr size_t MAX_IMAGE_QUEUE_SIZE = 100;

    // file paths
    FString SessionDir;
    FString ImageDir;

    // file streams
    std::unique_ptr<std::ofstream> ImuFile;
    std::unique_ptr<std::ofstream> GpsFile;
    std::unique_ptr<std::ofstream> GtImuFile;
    std::unique_ptr<std::ofstream> TimesFile;
    std::unique_ptr<std::ofstream> BarometerFile;
    std::unique_ptr<std::ofstream> MagnetometerFile;

    // worker functions
    void SimulationWorker();  // Master simulation loop with perfect timing
    void WriterWorker();
    void ImageWriterWorker();


    // sensor capture functions (called at precise intervals)
    void CaptureIMU(uint64_t timestamp_ns);
    void CaptureGPS(uint64_t timestamp_ns);
    void CaptureGroundTruth(uint64_t timestamp_ns);
    void CaptureImage(uint64_t timestamp_ns);
    void CaptureBarometer(uint64_t timestamp_ns);
    void CaptureMagnetometer(uint64_t timestamp_ns);


    // weather control functions
    void UpdateWeatherSystem();
    void SetWeatherParameters(float fog, float rain, float snow, float leaves, float roadLeaves);
    void SetTimeOfDay();


    // helper functions
    bool EnsureDir(const FString& Path) const;
    FString MakeSessionDir() const;
    void OpenFiles();
    void CloseFiles();
    void CreateCalibrationFiles();
    void CreateCameraImuConfigFile();
    void CreateImuSettingsFile();
    bool JoinThreadWithTimeout(std::thread& thread, const TCHAR* Name, int TimeoutSec);
    bool WriteImageFile(const FString& FilePath, const std::vector<uint8_t>& ImageData);
};


