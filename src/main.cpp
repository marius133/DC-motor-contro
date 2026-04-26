#include <Arduino.h>

#include <cctype>
#include <cstring>

#include "BME680Sensor.h"
#include "CsvLogger.h"
#include "GpsSensor.h"
#include "motorSpeedController.h"
#include "rc_input.h"

namespace
{
struct DriveMotorConfig
{
    MotorDriver::DriverPins driverPins;
    MotorDriver::DriverConfig driver;
    uint8_t encoderPinA = 0;
    uint8_t encoderPinB = 1;
    EncoderDriver::config encoder = {2048, false, 0.2f};
    MotorSpeedController::SpeedControlConfig speedControl;
};

struct AppConfig
{
    struct LoggingConfig
    {
        uint8_t manualTriggerChannel = 6;
        uint8_t autoIntervalChannel = 7;
        uint16_t switchOnThreshold = 1500;
        uint32_t autoIntervalMs = 5000;
    };

    struct DriveTrimConfig
    {
        float leftTargetScale = 1.0f;
        float rightTargetScale = 1.0f;
    };

    struct StraightDriveSyncConfig
    {
        bool enabled = true;
        float steerDeadband = 0.08f;
        float minThrottle = 0.0f;
        float gain = 0.25f;
        float maxCorrection = 0.20f;
    };

    uint32_t usbBaudrate = 115200;
    uint32_t gpsBaudrate = 9600;
    uint32_t bmeReadIntervalMs = 1000;
    uint32_t statusPrintIntervalMs = 500;
    uint8_t sdChipSelect = BUILTIN_SDCARD;
    const char* logFilename = "env_log.csv";
    BME680Sensor::Config bme680;
    LoggingConfig logging;
    DriveTrimConfig driveTrim;
    StraightDriveSyncConfig straightSync;
    RCInput::Config rcInput;
    DriveMotorConfig leftMotor;
    DriveMotorConfig rightMotor;
};

MotorDriver::DriverPins makeMotorPins(uint8_t en1, uint8_t en2, uint8_t pwm1, uint8_t pwm2)
{
    MotorDriver::DriverPins pins{};
    pins.EN1 = en1;
    pins.EN2 = en2;
    pins.PWM1 = pwm1;
    pins.PWM2 = pwm2;
    return pins;
}

MotorDriver::DriverConfig makeMotorConfig(bool invert = false,
                                          float minimumForwardDriveCommand = 0.0f,
                                          float minimumReverseDriveCommand = 0.0f)
{
    MotorDriver::DriverConfig cfg;
    cfg.invert = invert;
    cfg.deadzone = 0.05f;
    cfg.minimumForwardDriveCommand = minimumForwardDriveCommand;
    cfg.minimumReverseDriveCommand = minimumReverseDriveCommand;
    return cfg;
}

EncoderDriver::config makeEncoderConfig(int32_t countsPerRev, bool invert = false)
{
    EncoderDriver::config cfg{};
    cfg.counts_per_rev = countsPerRev;
    cfg.invert = invert;
    cfg.l_pass_alpha = 0.2f;
    return cfg;
}

MotorSpeedController::SpeedControlConfig makeSpeedControlConfig()
{
    MotorSpeedController::SpeedControlConfig cfg;
    cfg.maxTargetSpeedCountsPerSec = 4000.0f;
    cfg.kp = 0.00025f;
    cfg.ki = 0.00015f;
    cfg.kd = 0.0f;
    cfg.kf = 0.00020f;
    cfg.integralLimit = 0.4f;
    return cfg;
}

AppConfig makeAppConfig()
{
    AppConfig cfg;

    // Endre pinner og viktige parametre her.
    cfg.usbBaudrate = 115200;
    cfg.gpsBaudrate = 9600;
    cfg.bmeReadIntervalMs = 1000;
    cfg.statusPrintIntervalMs = 500;
    cfg.sdChipSelect = BUILTIN_SDCARD;
    cfg.logFilename = "env_log.csv";

    cfg.bme680.address = 0x76;
    cfg.bme680.temperatureOffsetC = 0.0f;
    cfg.bme680.heaterTempC = 320;
    cfg.bme680.heaterTimeMs = 150;

    cfg.logging.manualTriggerChannel = 6;
    cfg.logging.autoIntervalChannel = 7;
    cfg.logging.switchOnThreshold = 1500;
    cfg.logging.autoIntervalMs = 5000;

    // Static trim. Keep both at 1.0 when straightSync is enabled unless one
    // side needs a known mechanical offset.
    cfg.driveTrim.leftTargetScale = 1.0f;
    cfg.driveTrim.rightTargetScale = 1.0f;

    // Closed-loop straight driving in both forward and reverse. When steering
    // is centered, encoder speed difference keeps both wheels at the same speed.
    cfg.straightSync.enabled = true;
    cfg.straightSync.steerDeadband = 0.08f;
    // RCInput applies its own deadzone, so 0.0 means sync applies for every
    // actual forward/reverse command, including very low throttle.
    cfg.straightSync.minThrottle = 0.0f;
    cfg.straightSync.gain = 0.40f;
    cfg.straightSync.maxCorrection = 0.30f;

    cfg.rcInput.channelMin = 172;
    cfg.rcInput.channelMid = 992;
    cfg.rcInput.channelMax = 1811;
    cfg.rcInput.throttleChannel = 2;
    cfg.rcInput.steerChannel = 4;
    cfg.rcInput.armChannel = 5;
    cfg.rcInput.armThreshold = 1500;
    cfg.rcInput.failsafeTimeoutMs = 100;
    cfg.rcInput.deadzone = 0.05f;

    cfg.leftMotor.driverPins = makeMotorPins(23, 22, 2, 3);
    cfg.leftMotor.driver = makeMotorConfig(false, 0.0f, 0.0f);
    cfg.leftMotor.encoderPinA = 6;
    cfg.leftMotor.encoderPinB = 7;
    cfg.leftMotor.encoder = makeEncoderConfig(2048, false);
    cfg.leftMotor.speedControl = makeSpeedControlConfig();

    cfg.rightMotor.driverPins = makeMotorPins(21, 20, 4, 5);
    // Right motor wiring was reversed relative to the commanded direction, so
    // invert the driver command to keep forward/reverse and pivot steering
    // aligned with the RC inputs.
    cfg.rightMotor.driver = makeMotorConfig(true, 0.0f, 0.0f);
    cfg.rightMotor.encoderPinA = 8;
    cfg.rightMotor.encoderPinB = 9;
    cfg.rightMotor.encoder = makeEncoderConfig(2048, false);
    cfg.rightMotor.speedControl = makeSpeedControlConfig();

    return cfg;
}

const AppConfig kConfig = makeAppConfig();

HardwareSerial& kRcSerial = Serial6;
HardwareSerial& kGpsSerial = Serial2;

RCInput rcInput;
BME680Sensor bme680Sensor;
GpsSensor gpsSensor;
CsvLogger csvLogger;
MotorSpeedController leftMotor(kConfig.leftMotor.driverPins,
                               kConfig.leftMotor.driver,
                               kConfig.leftMotor.encoderPinA,
                               kConfig.leftMotor.encoderPinB,
                               kConfig.leftMotor.encoder,
                               kConfig.leftMotor.speedControl);
MotorSpeedController rightMotor(kConfig.rightMotor.driverPins,
                                kConfig.rightMotor.driver,
                                kConfig.rightMotor.encoderPinA,
                                kConfig.rightMotor.encoderPinB,
                                kConfig.rightMotor.encoder,
                                kConfig.rightMotor.speedControl);

bool sdReady = false;
elapsedMillis bmeTimer;
elapsedMillis autoLogTimer;
elapsedMillis statusTimer;
elapsedMicros motorControlTimer;
bool manualLogSwitchLatched = false;
bool autoLoggingWasEnabled = false;

enum class MotorTestSide
{
    None,
    Left,
    Right
};

struct MotorTestState
{
    MotorTestSide side = MotorTestSide::None;
    float command = 0.0f;
    bool active = false;
    elapsedMillis timer;
};

constexpr uint32_t kMotorTestDurationMs = 1000;
constexpr float kMotorTestCommand = 0.35f;
constexpr float kNeutralDriveCommandDeadband = 0.001f;
MotorTestState motorTest;
char serialCommandBuffer[16] = {};
size_t serialCommandLength = 0;

void initializeSerial()
{
    Serial.begin(kConfig.usbBaudrate);
    while (!Serial && millis() < 4000)
    {
        delay(10);
    }

    Serial.println("System startup");
    Serial.println("CRSF receiver on Serial6");
    Serial.println("SAM-M10Q GPS on Serial2");
}

void initializeSensors()
{
    if (bme680Sensor.begin(kConfig.bme680))
    {
        Serial.print("BME680 initialized on ");
        Serial.print(bme680Sensor.detectedBusName());
        Serial.print(" (0x");
        Serial.print(bme680Sensor.detectedAddress(), HEX);
        Serial.println(")");
    }
    else
    {
        Serial.println("BME680 not detected");
    }

    gpsSensor.begin(kGpsSerial, kConfig.gpsBaudrate);
    Serial.println("GPS serial initialized");
}

void initializeControl()
{
    rcInput.begin(kRcSerial, kConfig.rcInput);
    if (rcInput.isInitialized())
    {
        Serial.println("CRSF input initialized");
    }
    else
    {
        Serial.println("CRSF input initialization failed");
    }

    leftMotor.begin();
    rightMotor.begin();
    Serial.println("Motor drivers initialized");
}

void initializeLogging()
{
    sdReady = csvLogger.begin(kConfig.sdChipSelect, kConfig.logFilename);
    if (sdReady)
    {
        Serial.print("CSV logging enabled: ");
        Serial.println(csvLogger.filename());
        return;
    }

    Serial.println("SD card initialization failed");
}

void printMotorTestHelp()
{
    Serial.println("Motor test commands: lf/lr/rf/rr, stop, help");
    Serial.println("Example: lr runs left motor reverse for 1 second at 35% command");
}

void stopMotorTest(const char* reason)
{
    motorTest.active = false;
    motorTest.side = MotorTestSide::None;
    motorTest.command = 0.0f;
    leftMotor.enable(false);
    rightMotor.enable(false);

    Serial.print("Motor test stopped");
    if (reason != nullptr)
    {
        Serial.print(": ");
        Serial.print(reason);
    }
    Serial.println();
}

void startMotorTest(MotorTestSide side, float command, const char* label)
{
    motorTest.side = side;
    motorTest.command = command;
    motorTest.active = true;
    motorTest.timer = 0;

    leftMotor.enable(side == MotorTestSide::Left);
    rightMotor.enable(side == MotorTestSide::Right);

    Serial.print("Motor test ");
    Serial.print(label);
    Serial.println(" started");
}

void handleSerialCommand(char* command)
{
    for (size_t i = 0; command[i] != '\0'; ++i)
    {
        command[i] = static_cast<char>(tolower(command[i]));
    }

    if (strcmp(command, "lf") == 0)
    {
        startMotorTest(MotorTestSide::Left, kMotorTestCommand, "left forward");
    }
    else if (strcmp(command, "lr") == 0)
    {
        startMotorTest(MotorTestSide::Left, -kMotorTestCommand, "left reverse");
    }
    else if (strcmp(command, "rf") == 0)
    {
        startMotorTest(MotorTestSide::Right, kMotorTestCommand, "right forward");
    }
    else if (strcmp(command, "rr") == 0)
    {
        startMotorTest(MotorTestSide::Right, -kMotorTestCommand, "right reverse");
    }
    else if (strcmp(command, "stop") == 0)
    {
        stopMotorTest("serial command");
    }
    else if (strcmp(command, "help") == 0)
    {
        printMotorTestHelp();
    }
    else
    {
        Serial.print("Unknown command: ");
        Serial.println(command);
        printMotorTestHelp();
    }
}

void processSerialInput()
{
    while (Serial.available() > 0)
    {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            serialCommandBuffer[serialCommandLength] = '\0';
            if (serialCommandLength > 0)
            {
                handleSerialCommand(serialCommandBuffer);
            }
            serialCommandLength = 0;
            return;
        }

        if (isprint(static_cast<unsigned char>(c)) && serialCommandLength < (sizeof(serialCommandBuffer) - 1))
        {
            serialCommandBuffer[serialCommandLength++] = c;
        }
    }
}

void updateEnvironment()
{
    if (!bme680Sensor.isInitialized() || bmeTimer < kConfig.bmeReadIntervalMs)
    {
        return;
    }

    bmeTimer = 0;
    if (!bme680Sensor.read())
    {
        Serial.println("BME680 read failed");
    }
}

bool updateMotorTest(uint32_t dtUs)
{
    if (!motorTest.active)
    {
        return false;
    }

    if (motorTest.timer >= kMotorTestDurationMs)
    {
        stopMotorTest("complete");
        leftMotor.update(dtUs);
        rightMotor.update(dtUs);
        return true;
    }

    if (motorTest.side == MotorTestSide::Left)
    {
        if (!leftMotor.isEnabled())
        {
            leftMotor.enable(true);
        }
        if (rightMotor.isEnabled())
        {
            rightMotor.enable(false);
        }

        leftMotor.updateOpenLoop(dtUs, motorTest.command);
        rightMotor.update(dtUs);
        return true;
    }

    if (motorTest.side == MotorTestSide::Right)
    {
        if (!rightMotor.isEnabled())
        {
            rightMotor.enable(true);
        }
        if (leftMotor.isEnabled())
        {
            leftMotor.enable(false);
        }

        leftMotor.update(dtUs);
        rightMotor.updateOpenLoop(dtUs, motorTest.command);
        return true;
    }

    stopMotorTest("invalid side");
    return true;
}

void updateMotors(const RCCommand& rc)
{
    const uint32_t dtUs = motorControlTimer;
    motorControlTimer = 0;

    if (dtUs == 0)
    {
        return;
    }

    if (updateMotorTest(dtUs))
    {
        return;
    }

    if (!rc.valid || !rc.armed)
    {
        if (leftMotor.isEnabled())
        {
            leftMotor.enable(false);
            rightMotor.enable(false);
        }

        leftMotor.update(dtUs);
        rightMotor.update(dtUs);
        return;
    }

    if (fabsf(rc.throttle) <= kNeutralDriveCommandDeadband &&
        fabsf(rc.steer) <= kNeutralDriveCommandDeadband)
    {
        if (leftMotor.isEnabled() || rightMotor.isEnabled())
        {
            leftMotor.enable(false);
            rightMotor.enable(false);
        }

        leftMotor.update(dtUs);
        rightMotor.update(dtUs);
        return;
    }

    float leftTarget = 0.0f;
    float rightTarget = 0.0f;
    if (fabsf(rc.throttle) > kNeutralDriveCommandDeadband)
    {
        const float turnAmount = constrain(fabsf(rc.steer), 0.0f, 1.0f);
        const float innerWheelScale = 1.0f - turnAmount;

        // While driving forward/reverse, steering slows the inside wheel
        // instead of immediately commanding it to stop or reverse.
        if (rc.steer >= 0.0f)
        {
            leftTarget = rc.throttle;
            rightTarget = rc.throttle * innerWheelScale;
        }
        else
        {
            leftTarget = rc.throttle * innerWheelScale;
            rightTarget = rc.throttle;
        }
    }
    else
    {
        // With no forward/reverse command, keep pivot turning available.
        leftTarget = rc.steer;
        rightTarget = -rc.steer;
    }

    leftTarget *= kConfig.driveTrim.leftTargetScale;
    rightTarget *= kConfig.driveTrim.rightTargetScale;

    if (kConfig.straightSync.enabled &&
        fabsf(rc.steer) < kConfig.straightSync.steerDeadband &&
        fabsf(rc.throttle) > kConfig.straightSync.minThrottle)
    {
        // Compare wheel speeds in the commanded travel direction, so the same
        // sync correction works when driving forward and reverse.
        const float direction = (rc.throttle >= 0.0f) ? 1.0f : -1.0f;
        const float leftSpeedAlongCommand = leftMotor.measuredSpeedCountsPerSec() * direction;
        const float rightSpeedAlongCommand = rightMotor.measuredSpeedCountsPerSec() * direction;
        const float speedError = rightSpeedAlongCommand - leftSpeedAlongCommand;
        const float maxSpeed = fabsf(kConfig.leftMotor.speedControl.maxTargetSpeedCountsPerSec);
        const float correction = constrain((speedError / maxSpeed) * kConfig.straightSync.gain,
                                           -kConfig.straightSync.maxCorrection,
                                           kConfig.straightSync.maxCorrection);

        leftTarget += correction * direction;
        rightTarget -= correction * direction;
    }

    leftTarget = constrain(leftTarget, -1.0f, 1.0f);
    rightTarget = constrain(rightTarget, -1.0f, 1.0f);

    if (!leftMotor.isEnabled())
    {
        leftMotor.enable(true);
        rightMotor.enable(true);
    }

    leftMotor.setTargetNormalized(leftTarget);
    rightMotor.setTargetNormalized(rightTarget);
    leftMotor.update(dtUs);
    rightMotor.update(dtUs);
}

uint16_t rawChannelValue(const RCCommand& rc, uint8_t channel)
{
    switch (channel)
    {
        case 1: return rc.rawCh1;
        case 2: return rc.rawCh2;
        case 3: return rc.rawCh3;
        case 4: return rc.rawCh4;
        case 5: return rc.rawCh5;
        case 6: return rc.rawCh6;
        case 7: return rc.rawCh7;
        case 8: return rc.rawCh8;
        default: return kConfig.rcInput.channelMin;
    }
}

bool isLoggingChannelOn(const RCCommand& rc, uint8_t channel)
{
    if (!rc.valid)
    {
        return false;
    }

    return rawChannelValue(rc, channel) > kConfig.logging.switchOnThreshold;
}

bool appendLogRecord(const char* source)
{
    if (!sdReady)
    {
        Serial.println("SD log skipped: SD card not ready");
        return false;
    }

    LogRecord record{};
    record.deviceTimestampMs = millis();
    record.environment = bme680Sensor.getReading();
    record.gps = gpsSensor.getReading();

    if (!csvLogger.append(record))
    {
        Serial.println("Failed to append CSV record");
        return false;
    }

    Serial.print("SD log saved (");
    Serial.print(source);
    Serial.print("): ");
    Serial.print(csvLogger.filename());
    Serial.print(" at ");
    Serial.print(record.deviceTimestampMs);
    Serial.println(" ms");
    return true;
}

void logMeasurements(const RCCommand& rc)
{
    const bool autoLoggingEnabled = isLoggingChannelOn(rc, kConfig.logging.autoIntervalChannel);
    const bool manualLoggingEnabled = isLoggingChannelOn(rc, kConfig.logging.manualTriggerChannel);

    if (autoLoggingEnabled && !autoLoggingWasEnabled)
    {
        autoLogTimer = 0;
    }

    if (autoLoggingEnabled)
    {
        if (autoLogTimer >= kConfig.logging.autoIntervalMs)
        {
            appendLogRecord("auto");
            autoLogTimer = 0;
        }
    }
    else if (manualLoggingEnabled && !manualLogSwitchLatched)
    {
        appendLogRecord("manual");
    }

    manualLogSwitchLatched = manualLoggingEnabled;
    autoLoggingWasEnabled = autoLoggingEnabled;
}

void printStatus(const RCCommand& rc)
{
    const RCDebugStatus rcDebug = rcInput.getDebugStatus();

    Serial.print("RC valid=");
    Serial.print(rc.valid ? 1 : 0);
    Serial.print(" link=");
    Serial.print(rcDebug.linkUp ? 1 : 0);
    Serial.print(" armed=");
    Serial.print(rc.armed ? 1 : 0);
    Serial.print(" throttle=");
    Serial.print(rc.throttle, 3);
    Serial.print(" steer=");
    Serial.print(rc.steer, 3);
    Serial.print(" bytes=");
    Serial.print(rcDebug.rawBytesReceived);
    Serial.print(" pkts=");
    Serial.print(rcDebug.packetsReceived);
    Serial.print(" age_ms=");
    Serial.print(rcDebug.lastPacketAgeMs);
    Serial.print(" raw=[");
    Serial.print(rc.rawCh1);
    Serial.print(",");
    Serial.print(rc.rawCh2);
    Serial.print(",");
    Serial.print(rc.rawCh3);
    Serial.print(",");
    Serial.print(rc.rawCh4);
    Serial.print(",");
    Serial.print(rc.rawCh5);
    Serial.print(",");
    Serial.print(rc.rawCh6);
    Serial.print(",");
    Serial.print(rc.rawCh7);
    Serial.print(",");
    Serial.print(rc.rawCh8);
    Serial.print("]");
    Serial.print(" | left_spd=");
    Serial.print(leftMotor.measuredSpeedCountsPerSec(), 0);
    Serial.print("/");
    Serial.print(leftMotor.targetSpeedCountsPerSec(), 0);
    Serial.print(" right_spd=");
    Serial.print(rightMotor.measuredSpeedCountsPerSec(), 0);
    Serial.print("/");
    Serial.print(rightMotor.targetSpeedCountsPerSec(), 0);
    Serial.print(" cmd=");
    Serial.print(leftMotor.lastCommand(), 2);
    Serial.print("/");
    Serial.print(rightMotor.lastCommand(), 2);
    Serial.print(" | log_mode=");
    if (isLoggingChannelOn(rc, kConfig.logging.autoIntervalChannel))
    {
        Serial.print("auto");
    }
    else if (isLoggingChannelOn(rc, kConfig.logging.manualTriggerChannel))
    {
        Serial.print("manual");
    }
    else
    {
        Serial.print("idle");
    }

    const auto& bmeReading = bme680Sensor.getReading();
    if (bmeReading.valid)
    {
        Serial.print(" | temp=");
        Serial.print(bmeReading.temperatureC, 1);
        Serial.print("C hum=");
        Serial.print(bmeReading.humidityPct, 1);
        Serial.print("% press=");
        Serial.print(bmeReading.pressurehPa, 1);
        Serial.print("hPa gas=");
        Serial.print(bmeReading.gasResistanceOhm, 0);
        Serial.print("ohm");
    }

    if (!gpsSensor.isDataValid())
    {
        Serial.print(" | gps_fix=0");
        return;
    }

    const GPSReading& gps = gpsSensor.getReading();
    Serial.print(" | lat=");
    Serial.print(gps.latitudeDeg, 6);
    Serial.print(" lon=");
    Serial.print(gps.longitudeDeg, 6);
    Serial.print(" sats=");
    Serial.print(gps.satellites);
}
} // namespace

void setup()
{
    initializeSerial();
    initializeSensors();
    initializeControl();
    initializeLogging();
}

void loop()
{
    processSerialInput();
    rcInput.update();
    gpsSensor.update();
    updateEnvironment();

    const RCCommand rc = rcInput.getCommand();
    updateMotors(rc);
    logMeasurements(rc);

    if (statusTimer >= kConfig.statusPrintIntervalMs)
    {
        statusTimer = 0;
        printStatus(rc);
        Serial.println();
    }
}
