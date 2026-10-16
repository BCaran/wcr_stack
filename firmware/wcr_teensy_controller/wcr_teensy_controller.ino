/*
 * wcr_teensy_controller.ino
 *
 * Teensy 4.0 thruster and load-cell controller for the wall-climbing robot.
 *
 * Changes from the PWM version (2026-10-09)
 *   - The four ESC outputs are DShot300 instead of 50 Hz servo PWM (pins 3..6).
 *   - Thruster commands are fractions 0.00..1.00 (0 = stop, 1 = full throttle)
 *     instead of 0..180. Inside the firmware they are integer thousandths.
 *   - The ramp-down logic is the same as before. Its default (0.001 every 9 ms)
 *     gives the same slope as the old 1/180 every 50 ms: full scale in 9 s.
 *   - New ESC frame with the thruster command state, the raw telemetry of one
 *     ESC and the raw current ADC. One frame is sent for every telemetry
 *     request. Requests are spaced 25 ms apart until all four ESCs have
 *     answered steadily for 2 s, and are then sent as fast as the ESCs answer.
 *   - The DATA frame carries load-cell data only. The applied and target
 *     thruster commands moved to the ESC frame.
 *   - Nothing is converted on the Teensy. Scaling to RPM, volts and amps is done
 *     on the PC.
 *   - New error bit 8 (ERROR_ESC_TELEMETRY_STALE).
 *
 * Wiring used by the new parts
 *   Teensy pin 0 (RX1)  <- ESC telemetry pad (shared by the four ESCs)
 *   Teensy pin 22 (A8)  <- ESC current output
 *   Pin 1 (TX1) is claimed by Serial1 but not used.
 *
 * BLHeliSuite32: "Auto Telemetry" must be OFF. The firmware asks one ESC at a
 * time; with Auto Telemetry on, the four ESCs collide on the shared wire.
 *
 * Commands from the PC (ASCII_PAYLOAD*CRC16\n, unchanged framing)
 *   PWM,seq,t1,t2,t3,t4[,stale]   thruster commands, fractions 0.00..1.00
 *   CFG,seq,min,max,startup,timeout_ms,ramp_step,ramp_interval_ms
 *                                 min, max, startup and ramp_step are fractions.
 *                                 ramp_step is 0 (no ramp) or a multiple of 0.001
 *   TARE,seq                      unchanged
 *   CAL,seq,front,rear            unchanged
 *
 * Frames to the PC
 *   DATA,seq,teensy_ms,front_raw,rear_raw,front_tared,rear_tared,front_g,rear_g,
 *        serial_timeout,error_flags
 *     Load cells only. Sent for every new pair of HX711 samples (10 or 80 Hz,
 *     set by the HX711).
 *
 *   ESC,seq,teensy_ms,motor,status,temp,voltage,current,consumption,erpm,
 *       applied,target,dshot,current_adc
 *     motor        1..4
 *     status       1 = valid answer, 0 = no answer (timeout), 2 = corrupt answer
 *     temp         ESC temperature, raw [deg C]
 *     voltage      raw, unit 0.01 V
 *     current      raw, unit 0.01 A (always 0 on this ESC)
 *     consumption  raw [mAh]       (always 0 on this ESC)
 *     erpm         raw, unit 100 electrical RPM. rpm = erpm * 100 / pole_pairs
 *     applied      command currently applied to this motor, fraction 0.000..1.000
 *     target       latest command received for this motor, fraction 0.000..1.000
 *     dshot        DShot value being sent to this motor (0, or 48..2047)
 *     current_adc  raw 12-bit ADC count of the current pin (0..4095 = 0..3.3 V),
 *                  total of all four motors. The pin is sampled every 10 ms;
 *                  each frame carries the latest sample
 *     The five telemetry fields (temp .. erpm) are 0 unless status is 1.
 *     applied, target, dshot and current_adc come from the Teensy itself and
 *     are valid whatever the status.
 */

#include <Arduino.h>
#include <HX711.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------

// Front HX711: DAT = 9, SCK = 10
constexpr uint8_t FRONT_HX711_DAT_PIN = 9;
constexpr uint8_t FRONT_HX711_SCK_PIN = 10;

// Rear HX711: DAT = 11, SCK = 12
constexpr uint8_t REAR_HX711_DAT_PIN = 11;
constexpr uint8_t REAR_HX711_SCK_PIN = 12;

// ESC signal pins requested for the robot.
#define ESC_pin1 3
#define ESC_pin2 4
#define ESC_pin3 5
#define ESC_pin4 6

constexpr size_t THRUSTER_COUNT = 4;
constexpr uint8_t ESC_SIGNAL_PINS[THRUSTER_COUNT] = {
    ESC_pin1, ESC_pin2, ESC_pin3, ESC_pin4};

constexpr uint8_t STATUS_LED_PIN = 13;
constexpr uint32_t LED_OK_TOGGLE_INTERVAL_MS = 500;
constexpr uint32_t LED_ERROR_TOGGLE_INTERVAL_MS = 200;

// ESC telemetry wire, shared by the four ESCs, on Serial1 RX = pin 0.
#define ESC_TELEMETRY_SERIAL Serial1
constexpr uint32_t ESC_TELEMETRY_BAUD = 115200;

// ESC analog current output (total of all four motors) on pin 22 = A8.
constexpr uint8_t ESC_CURRENT_PIN = 22;

// -----------------------------------------------------------------------------
// Thruster command scale and DShot output.
// On the serial link a thruster command is a fraction 0.00..1.00. Inside the
// firmware it is an integer number of thousandths, 0..1000.
//   command 0        -> DShot 0 (motor stop)
//   command 1..1000  -> DShot 50..2047 (throttle range is 48..2047)
// One DShot300 frame is sent to all four ESCs every millisecond from a timer
// interrupt, so the ESCs keep their signal even while loop() is blocked.
// -----------------------------------------------------------------------------

constexpr uint16_t THRUSTER_COMMAND_FULL_SCALE = 1000;

constexpr uint32_t DSHOT_BITRATE = 300000;
constexpr uint32_t DSHOT_FRAME_PERIOD_US = 1000;
constexpr uint16_t DSHOT_STOP = 0;
constexpr uint16_t DSHOT_THROTTLE_MIN = 48;  // values 1..47 are ESC commands
constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;

// -----------------------------------------------------------------------------
// ESC telemetry and current measurement. Values are forwarded raw.
// -----------------------------------------------------------------------------

// The four ESCs share one telemetry wire, and an ESC that is asked while it is
// busy (start-up or arming tones) does not answer in time. Asking the others
// at full speed in that situation left random ESCs unresponsive after power-up
// on this hardware. Requests are therefore spaced like the bench-test sketch,
// which starts the ESCs reliably, until every ESC has answered steadily for
// ESC_TELEMETRY_FAST_AFTER_MS. Only then is the next ESC asked as soon as the
// previous one has answered. Any missing or corrupt answer switches back to
// the slow spacing.
constexpr uint32_t ESC_TELEMETRY_SLOW_REQUEST_INTERVAL_MS = 25;
constexpr uint32_t ESC_TELEMETRY_FAST_REQUEST_INTERVAL_MS = 0;
constexpr uint32_t ESC_TELEMETRY_FAST_AFTER_MS = 2000;
// Long on purpose: a late answer must never be mistaken for the next ESC's answer.
constexpr uint32_t ESC_TELEMETRY_RESPONSE_TIMEOUT_MS = 100;
constexpr uint32_t ESC_TELEMETRY_STALE_MS = 500;
// No telemetry request until the ESCs have seen plain zero-throttle frames for
// this long after the first DShot frame. A request while an ESC is still
// arming can leave it unarmed. Betaflight waits 5 s for the same reason.
constexpr uint32_t ESC_TELEMETRY_START_DELAY_MS = 5000;
constexpr size_t ESC_TELEMETRY_FRAME_SIZE = 10;

constexpr uint8_t ESC_STATUS_NO_ANSWER = 0;
constexpr uint8_t ESC_STATUS_VALID = 1;
constexpr uint8_t ESC_STATUS_CORRUPT = 2;

constexpr uint8_t ADC_RESOLUTION_BITS = 12;

// The current pin has a high source resistance (about 65 kOhm). Every ADC
// conversion takes a little charge from it, so converting fast shifts the
// reading upwards. The pin is therefore sampled at a fixed slow rate:
// 100 samples/s x 4 conversions = 400 conversions/s, about the load it had
// during the calibration (512 conversions/s). Do not raise these without
// repeating the current calibration.
constexpr uint32_t ESC_CURRENT_SAMPLE_INTERVAL_MS = 10;
constexpr uint8_t ADC_HARDWARE_AVERAGING = 4;

// -----------------------------------------------------------------------------
// Defaults. ROS 2 sends the runtime configuration when the bridge starts.
// Calibration factors are HX711 counts per gram and may be negative.
// -----------------------------------------------------------------------------

constexpr float DEFAULT_FRONT_COUNTS_PER_GRAM = 1.0f;
constexpr float DEFAULT_REAR_COUNTS_PER_GRAM = 1.0f;

// Thruster limits in thousandths of full throttle (1000 = 1.00).
constexpr uint16_t DEFAULT_THRUSTER_MIN_COMMAND = 0;
constexpr uint16_t DEFAULT_THRUSTER_MAX_COMMAND = THRUSTER_COMMAND_FULL_SCALE;
constexpr uint16_t DEFAULT_STARTUP_THRUSTER_COMMAND = 0;

// This timeout only reports a communication error. It never changes PWM.
constexpr uint32_t DEFAULT_COMMAND_TIMEOUT_MS = 500;

// Controlled deceleration: decrease by one step every interval. The default,
// 0.001 every 9 ms, has the same slope as the previous implementation
// (1/180 every 50 ms): full scale in 9 s. Set step to 0 for an immediate decrease.
constexpr uint16_t DEFAULT_RAMP_DOWN_STEP_COMMAND = 1;
constexpr uint32_t DEFAULT_RAMP_DOWN_INTERVAL_MS = 9;

constexpr uint8_t TARE_SAMPLE_COUNT = 20;
constexpr bool AUTO_TARE_ON_STARTUP = true;
// Test switch: false runs the firmware with the HX711s untouched (no DATA frames,
// TARE answers OK without measuring). Keep true for normal use.
constexpr bool ENABLE_LOAD_CELLS = true;
constexpr uint32_t ESC_STARTUP_DELAY_MS = 1000;
constexpr uint32_t HX711_READY_TIMEOUT_MS = 1000;
constexpr uint32_t TRANSIENT_ERROR_DISPLAY_MS = 2000;

constexpr size_t RX_BUFFER_SIZE = 256;
constexpr size_t TX_PAYLOAD_SIZE = 256;

// Error bits published to ROS and used by the LED heartbeat.
enum ErrorFlag : uint16_t
{
    ERROR_NONE = 0,
    ERROR_SERIAL_COMMAND_TIMEOUT = 1U << 0,
    ERROR_ROS_COMMAND_STALE = 1U << 1,
    ERROR_FRONT_HX711_TIMEOUT = 1U << 2,
    ERROR_REAR_HX711_TIMEOUT = 1U << 3,
    ERROR_TARE_TIMEOUT = 1U << 4,
    ERROR_BAD_SERIAL_FRAME = 1U << 5,
    ERROR_RX_BUFFER_OVERFLOW = 1U << 6,
    ERROR_BAD_COMMAND = 1U << 7,
    ERROR_ESC_TELEMETRY_STALE = 1U << 8
};

struct DshotPin
{
    volatile uint32_t* setRegister;
    volatile uint32_t* clearRegister;
    uint32_t mask;
};

HX711 frontLoadCell;
HX711 rearLoadCell;

IntervalTimer dshotTimer;
DshotPin dshotPins[THRUSTER_COUNT];
uint32_t dshotBitCycles = 0;
uint32_t dshotZeroHighCycles = 0;
uint32_t dshotOneHighCycles = 0;

// Written by loop(), read by the DShot timer interrupt.
volatile uint16_t dshotValue[THRUSTER_COUNT] = {DSHOT_STOP, DSHOT_STOP, DSHOT_STOP, DSHOT_STOP};
volatile int8_t escTelemetryRequestMotor = -1;  // -1 = no request pending

uint8_t escTelemetryBuffer[ESC_TELEMETRY_FRAME_SIZE];
size_t escTelemetryLength = 0;
size_t escTelemetryMotor = 0;
bool escTelemetryWaiting = false;
uint32_t escTelemetryRequestMs = 0;
uint32_t dshotStartMs = 0;
uint32_t escTelemetrySequence = 0;
uint32_t escLastValidMs[THRUSTER_COUNT] = {0, 0, 0, 0};
bool escAnswering[THRUSTER_COUNT] = {false, false, false, false};  // last request answered validly
bool escAllAnswering = false;
uint32_t escAllAnsweringSinceMs = 0;
bool escFramesEnabled = false;  // ESC frames start after STATUS,BOOT
uint16_t escCurrentAdc = 0;     // latest sample of the current pin
uint32_t lastEscCurrentSampleMs = 0;

float frontCountsPerGram = DEFAULT_FRONT_COUNTS_PER_GRAM;
float rearCountsPerGram = DEFAULT_REAR_COUNTS_PER_GRAM;

int32_t frontTareOffset = 0;
int32_t rearTareOffset = 0;

uint16_t thrusterMinCommand = DEFAULT_THRUSTER_MIN_COMMAND;
uint16_t thrusterMaxCommand = DEFAULT_THRUSTER_MAX_COMMAND;
uint16_t startupThrusterCommand = DEFAULT_STARTUP_THRUSTER_COMMAND;
uint32_t commandTimeoutMs = DEFAULT_COMMAND_TIMEOUT_MS;
uint16_t rampDownStepCommand = DEFAULT_RAMP_DOWN_STEP_COMMAND;
uint32_t rampDownIntervalMs = DEFAULT_RAMP_DOWN_INTERVAL_MS;

uint16_t targetThrusterCommand[THRUSTER_COUNT] = {
    DEFAULT_STARTUP_THRUSTER_COMMAND,
    DEFAULT_STARTUP_THRUSTER_COMMAND,
    DEFAULT_STARTUP_THRUSTER_COMMAND,
    DEFAULT_STARTUP_THRUSTER_COMMAND};

uint16_t appliedThrusterCommand[THRUSTER_COUNT] = {
    DEFAULT_STARTUP_THRUSTER_COMMAND,
    DEFAULT_STARTUP_THRUSTER_COMMAND,
    DEFAULT_STARTUP_THRUSTER_COMMAND,
    DEFAULT_STARTUP_THRUSTER_COMMAND};

uint32_t lastPwmCommandMs = 0;
uint32_t lastRampDownUpdateMs = 0;
bool receivedPwmCommand = false;
bool rosCommandStale = false;
uint32_t sensorSequence = 0;

uint32_t lastFrontReadyMs = 0;
uint32_t lastRearReadyMs = 0;

uint16_t persistentErrorFlags = ERROR_NONE;
uint16_t transientErrorFlags = ERROR_NONE;
uint32_t transientErrorClearMs = 0;

bool statusLedState = false;
uint32_t lastLedToggleMs = 0;

char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
bool tareInProgress = false;

void readSerialInput();
void updateStatusLed();
void updateCommunicationHealth();
void updateThrusterOutputs();
void updateEscTelemetry();

// -----------------------------------------------------------------------------
// Error and LED handling
// -----------------------------------------------------------------------------

void setPersistentError(uint16_t flag, bool active)
{
    if (active)
    {
        persistentErrorFlags |= flag;
    }
    else
    {
        persistentErrorFlags &= static_cast<uint16_t>(~flag);
    }
}

void recordTransientError(uint16_t flag)
{
    transientErrorFlags |= flag;
    transientErrorClearMs = millis() + TRANSIENT_ERROR_DISPLAY_MS;
}

void updateTransientErrors()
{
    if (transientErrorFlags != ERROR_NONE &&
        static_cast<int32_t>(millis() - transientErrorClearMs) >= 0)
    {
        transientErrorFlags = ERROR_NONE;
    }
}

uint16_t currentErrorFlags()
{
    updateTransientErrors();
    return static_cast<uint16_t>(persistentErrorFlags | transientErrorFlags);
}

void updateStatusLed()
{
    const uint32_t now = millis();
    const bool hasError = currentErrorFlags() != ERROR_NONE;
    const uint32_t interval = hasError
                                  ? LED_ERROR_TOGGLE_INTERVAL_MS
                                  : LED_OK_TOGGLE_INTERVAL_MS;

    if (static_cast<uint32_t>(now - lastLedToggleMs) >= interval)
    {
        lastLedToggleMs = now;
        statusLedState = !statusLedState;
        digitalWrite(STATUS_LED_PIN, statusLedState ? HIGH : LOW);
    }
}

void responsiveDelay(uint32_t durationMs)
{
    const uint32_t startMs = millis();
    while (static_cast<uint32_t>(millis() - startMs) < durationMs)
    {
        readSerialInput();
        updateEscTelemetry();
        updateStatusLed();
        delay(1);
    }
}

// -----------------------------------------------------------------------------
// CRC and framed serial protocol
// Frame format: ASCII_PAYLOAD*ABCD\n, where ABCD is CRC-16/CCITT-FALSE.
// -----------------------------------------------------------------------------

uint16_t crc16Ccitt(const char* data)
{
    uint16_t crc = 0xFFFF;

    while (*data != '\0')
    {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(*data++)) << 8;

        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

void sendFramedPayload(const char* payload)
{
    const uint16_t crc = crc16Ccitt(payload);
    Serial.print(payload);
    Serial.print('*');

    char crcText[5];
    snprintf(crcText, sizeof(crcText), "%04X", crc);
    Serial.println(crcText);
}

void sendAck(const char* command, uint32_t commandSequence)
{
    char payload[TX_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload), "ACK,%s,%lu,OK", command,
             static_cast<unsigned long>(commandSequence));
    sendFramedPayload(payload);
}

void sendError(const char* command, uint32_t commandSequence, const char* reason)
{
    recordTransientError(ERROR_BAD_COMMAND);

    char payload[TX_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload), "ERR,%s,%lu,%s", command,
             static_cast<unsigned long>(commandSequence), reason);
    sendFramedPayload(payload);
}

bool validateAndStripCrc(char* line)
{
    char* separator = strrchr(line, '*');
    if (separator == nullptr)
    {
        return false;
    }

    *separator = '\0';
    const char* receivedCrcText = separator + 1;

    if (strlen(receivedCrcText) != 4)
    {
        return false;
    }

    char* endPtr = nullptr;
    const unsigned long receivedCrc = strtoul(receivedCrcText, &endPtr, 16);
    if (endPtr == receivedCrcText || *endPtr != '\0')
    {
        return false;
    }

    return static_cast<uint16_t>(receivedCrc) == crc16Ccitt(line);
}

// -----------------------------------------------------------------------------
// DShot output
// -----------------------------------------------------------------------------

// 16-bit DShot frame: 11-bit value, 1 telemetry-request bit, 4-bit checksum.
uint16_t makeDshotFrame(uint16_t value, bool requestTelemetry)
{
    const uint16_t packet =
        static_cast<uint16_t>((value << 1) | (requestTelemetry ? 1U : 0U));
    const uint16_t checksum =
        static_cast<uint16_t>((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0FU);
    return static_cast<uint16_t>((packet << 4) | checksum);
}

static inline void waitForCycle(uint32_t startCycle, uint32_t offsetCycles)
{
    while (static_cast<uint32_t>(ARM_DWT_CYCCNT - startCycle) < offsetCycles)
    {
    }
}

// Timer interrupt: send one frame to all four ESCs at the same time. The frame
// is generated by direct pin timing against the CPU cycle counter, with
// interrupts masked for about 55 us.
void dshotSendFrames()
{
    uint16_t frames[THRUSTER_COUNT];
    const int8_t request = escTelemetryRequestMotor;

    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        frames[i] = makeDshotFrame(dshotValue[i], request == static_cast<int8_t>(i));
    }

    // The telemetry-request bit goes out in exactly one frame.
    if (request >= 0)
    {
        escTelemetryRequestMotor = -1;
    }

    noInterrupts();
    const uint32_t startCycle = ARM_DWT_CYCCNT;
    uint32_t bitStart = 0;

    for (int8_t bit = 15; bit >= 0; --bit)
    {
        const uint16_t bitMask = static_cast<uint16_t>(1U << bit);

        // Every bit starts with a rising edge on all outputs.
        waitForCycle(startCycle, bitStart);
        for (size_t i = 0; i < THRUSTER_COUNT; ++i)
        {
            *dshotPins[i].setRegister = dshotPins[i].mask;
        }

        // A zero bit goes low early.
        waitForCycle(startCycle, bitStart + dshotZeroHighCycles);
        for (size_t i = 0; i < THRUSTER_COUNT; ++i)
        {
            if ((frames[i] & bitMask) == 0U)
            {
                *dshotPins[i].clearRegister = dshotPins[i].mask;
            }
        }

        // A one bit goes low late. Clearing a pin that is already low is harmless.
        waitForCycle(startCycle, bitStart + dshotOneHighCycles);
        for (size_t i = 0; i < THRUSTER_COUNT; ++i)
        {
            *dshotPins[i].clearRegister = dshotPins[i].mask;
        }

        bitStart += dshotBitCycles;
    }
    interrupts();
}

void beginDshot()
{
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        const uint8_t pin = ESC_SIGNAL_PINS[i];
        pinMode(pin, OUTPUT);
        digitalWriteFast(pin, LOW);

        dshotPins[i].setRegister = portSetRegister(pin);
        dshotPins[i].clearRegister = portClearRegister(pin);
        dshotPins[i].mask = digitalPinToBitMask(pin);
    }

    // DShot bit: a "1" is high for 75 % of the bit, a "0" for 37.5 %.
    dshotBitCycles = F_CPU_ACTUAL / DSHOT_BITRATE;
    dshotOneHighCycles = (dshotBitCycles * 3U) / 4U;
    dshotZeroHighCycles = (dshotBitCycles * 3U) / 8U;

    dshotStartMs = millis();
    dshotTimer.begin(dshotSendFrames, DSHOT_FRAME_PERIOD_US);
}

// Thruster command (thousandths, 0..1000) -> DShot value. 0 is motor stop;
// 1..1000 is spread linearly over the DShot throttle range and ends at 2047.
uint16_t thrusterCommandToDshot(uint16_t command)
{
    if (command == 0U)
    {
        return DSHOT_STOP;
    }
    if (command >= THRUSTER_COMMAND_FULL_SCALE)
    {
        return DSHOT_THROTTLE_MAX;
    }

    const uint32_t span = static_cast<uint32_t>(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN);
    const uint32_t scaled =
        (static_cast<uint32_t>(command) * span + THRUSTER_COMMAND_FULL_SCALE / 2U) /
        THRUSTER_COMMAND_FULL_SCALE;
    return static_cast<uint16_t>(DSHOT_THROTTLE_MIN + scaled);
}

// Internal thousandths -> fraction for the frames sent to the PC.
double commandToFraction(uint16_t command)
{
    return static_cast<double>(command) / static_cast<double>(THRUSTER_COMMAND_FULL_SCALE);
}

// -----------------------------------------------------------------------------
// Thrusters and communication health
// -----------------------------------------------------------------------------

uint16_t clampThrusterCommand(int32_t command)
{
    if (command < static_cast<int32_t>(thrusterMinCommand))
    {
        return thrusterMinCommand;
    }
    if (command > static_cast<int32_t>(thrusterMaxCommand))
    {
        return thrusterMaxCommand;
    }
    return static_cast<uint16_t>(command);
}

void writeAppliedThrusterCommand(size_t index, uint16_t value)
{
    appliedThrusterCommand[index] = clampThrusterCommand(value);
    dshotValue[index] = thrusterCommandToDshot(appliedThrusterCommand[index]);
}

void applyThrusterCommandsImmediate(const uint16_t commandValues[THRUSTER_COUNT])
{
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        const uint16_t clamped = clampThrusterCommand(commandValues[i]);
        targetThrusterCommand[i] = clamped;
        writeAppliedThrusterCommand(i, clamped);
    }
}

void setThrusterTargets(const uint16_t commandValues[THRUSTER_COUNT])
{
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        targetThrusterCommand[i] = clampThrusterCommand(commandValues[i]);

        // Increasing thrust is applied immediately. Only reductions are ramped.
        if (targetThrusterCommand[i] >= appliedThrusterCommand[i])
        {
            writeAppliedThrusterCommand(i, targetThrusterCommand[i]);
        }
    }
}

void updateThrusterOutputs()
{
    const uint32_t now = millis();

    // If the serial command stream is lost during a ramp, freeze the exact PWM
    // currently applied. The target is retained and ramping resumes only after a
    // new valid thruster-command frame proves that communication has recovered.
    if (receivedPwmCommand &&
        static_cast<uint32_t>(now - lastPwmCommandMs) > commandTimeoutMs)
    {
        lastRampDownUpdateMs = now;
        return;
    }

    // A zero step explicitly disables ramping and applies a reduction immediately.
    if (rampDownStepCommand == 0U)
    {
        for (size_t i = 0; i < THRUSTER_COUNT; ++i)
        {
            if (appliedThrusterCommand[i] != targetThrusterCommand[i])
            {
                writeAppliedThrusterCommand(i, targetThrusterCommand[i]);
            }
        }
        lastRampDownUpdateMs = now;
        return;
    }

    if (rampDownIntervalMs == 0U ||
        static_cast<uint32_t>(now - lastRampDownUpdateMs) < rampDownIntervalMs)
    {
        return;
    }

    const uint32_t elapsedMs = static_cast<uint32_t>(now - lastRampDownUpdateMs);
    const uint32_t elapsedSteps = elapsedMs / rampDownIntervalMs;
    lastRampDownUpdateMs += elapsedSteps * rampDownIntervalMs;

    const uint32_t maximumDecrease =
        elapsedSteps * static_cast<uint32_t>(rampDownStepCommand);

    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        if (appliedThrusterCommand[i] <= targetThrusterCommand[i])
        {
            continue;
        }

        const uint32_t difference =
            static_cast<uint32_t>(appliedThrusterCommand[i] - targetThrusterCommand[i]);
        const uint32_t decrease = min(difference, maximumDecrease);
        writeAppliedThrusterCommand(
            i, static_cast<uint16_t>(appliedThrusterCommand[i] - decrease));
    }
}

void applyStartupThrusterCommand()
{
    uint16_t values[THRUSTER_COUNT];
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        values[i] = startupThrusterCommand;
    }
    applyThrusterCommandsImmediate(values);
}

void updateCommunicationHealth()
{
    const bool serialCommandTimedOut =
        static_cast<uint32_t>(millis() - lastPwmCommandMs) > commandTimeoutMs;

    // Important: only report the error. Never modify applied thruster command here.
    setPersistentError(ERROR_SERIAL_COMMAND_TIMEOUT, serialCommandTimedOut);
    setPersistentError(ERROR_ROS_COMMAND_STALE, rosCommandStale);
}

// -----------------------------------------------------------------------------
// Load cells
// -----------------------------------------------------------------------------

bool tareBothLoadCells()
{
    if (!ENABLE_LOAD_CELLS)
    {
        return true;
    }

    tareInProgress = true;
    int64_t frontSum = 0;
    int64_t rearSum = 0;
    uint8_t frontSamples = 0;
    uint8_t rearSamples = 0;
    const uint32_t startMs = millis();

    // thruster outputs are deliberately left unchanged during tare.
    while (frontSamples < TARE_SAMPLE_COUNT || rearSamples < TARE_SAMPLE_COUNT)
    {
        if (frontSamples < TARE_SAMPLE_COUNT && frontLoadCell.is_ready())
        {
            frontSum += static_cast<int32_t>(frontLoadCell.read());
            ++frontSamples;
            lastFrontReadyMs = millis();
        }

        if (rearSamples < TARE_SAMPLE_COUNT && rearLoadCell.is_ready())
        {
            rearSum += static_cast<int32_t>(rearLoadCell.read());
            ++rearSamples;
            lastRearReadyMs = millis();
        }

        readSerialInput();
        updateEscTelemetry();
        updateCommunicationHealth();
        updateStatusLed();

        if (millis() - startMs > 10000U)
        {
            tareInProgress = false;
            lastRampDownUpdateMs = millis();
            setPersistentError(ERROR_TARE_TIMEOUT, true);
            return false;
        }
    }

    frontTareOffset = static_cast<int32_t>(frontSum / TARE_SAMPLE_COUNT);
    rearTareOffset = static_cast<int32_t>(rearSum / TARE_SAMPLE_COUNT);
    tareInProgress = false;
    lastRampDownUpdateMs = millis();
    setPersistentError(ERROR_TARE_TIMEOUT, false);
    return true;
}

float countsToGrams(int32_t taredCounts, float countsPerGram)
{
    if (!isfinite(countsPerGram) || fabsf(countsPerGram) < 1.0e-9f)
    {
        return NAN;
    }

    return static_cast<float>(taredCounts) / countsPerGram;
}

void updateHx711Health()
{
    if (!ENABLE_LOAD_CELLS)
    {
        return;
    }

    const uint32_t now = millis();

    if (frontLoadCell.is_ready())
    {
        lastFrontReadyMs = now;
    }
    if (rearLoadCell.is_ready())
    {
        lastRearReadyMs = now;
    }

    setPersistentError(
        ERROR_FRONT_HX711_TIMEOUT,
        static_cast<uint32_t>(now - lastFrontReadyMs) > HX711_READY_TIMEOUT_MS);

    setPersistentError(
        ERROR_REAR_HX711_TIMEOUT,
        static_cast<uint32_t>(now - lastRearReadyMs) > HX711_READY_TIMEOUT_MS);
}

void publishSensorDataIfReady()
{
    if (!ENABLE_LOAD_CELLS)
    {
        return;
    }

    if (!frontLoadCell.is_ready() || !rearLoadCell.is_ready())
    {
        return;
    }

    const int32_t frontRawAdc = static_cast<int32_t>(frontLoadCell.read());
    lastFrontReadyMs = millis();

    const int32_t rearRawAdc = static_cast<int32_t>(rearLoadCell.read());
    lastRearReadyMs = millis();

    const int32_t frontRawTared = frontRawAdc - frontTareOffset;
    const int32_t rearRawTared = rearRawAdc - rearTareOffset;

    const float frontGrams = countsToGrams(frontRawTared, frontCountsPerGram);
    const float rearGrams = countsToGrams(rearRawTared, rearCountsPerGram);

    const uint16_t errors = currentErrorFlags();

    char payload[TX_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload),
             "DATA,%lu,%lu,%ld,%ld,%ld,%ld,%.3f,%.3f,%u,%u",
             static_cast<unsigned long>(sensorSequence++),
             static_cast<unsigned long>(millis()),
             static_cast<long>(frontRawAdc),
             static_cast<long>(rearRawAdc),
             static_cast<long>(frontRawTared),
             static_cast<long>(rearRawTared),
             static_cast<double>(frontGrams),
             static_cast<double>(rearGrams),
             (errors & ERROR_SERIAL_COMMAND_TIMEOUT) != 0U ? 1U : 0U,
             errors);

    sendFramedPayload(payload);
}

// -----------------------------------------------------------------------------
// ESC telemetry (KISS / BLHeli_32 serial telemetry, 10 bytes, 115200 8N1)
//   0      temperature [deg C]
//   1..2   voltage     [0.01 V]
//   3..4   current     [0.01 A]
//   5..6   consumption [mAh]
//   7..8   electrical RPM / 100
//   9      CRC-8 (polynomial 0x07) over bytes 0..8
// The frame carries no ESC number. The firmware knows who answered because it
// only asks one ESC at a time. Every request ends in one ESC frame to the PC,
// with the values exactly as the ESC sent them.
// -----------------------------------------------------------------------------

uint8_t escTelemetryCrc8(const uint8_t* data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x80U) != 0U
                      ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                      : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

// status is ESC_STATUS_VALID, ESC_STATUS_NO_ANSWER or ESC_STATUS_CORRUPT.
// frame is only read when the status is ESC_STATUS_VALID.
void publishEscFrame(size_t motor, uint8_t status, const uint8_t* frame)
{
    if (!escFramesEnabled)
    {
        return;
    }

    unsigned temperature = 0;
    unsigned voltage = 0;
    unsigned current = 0;
    unsigned consumption = 0;
    unsigned erpm = 0;

    if (status == ESC_STATUS_VALID)
    {
        temperature = frame[0];
        voltage = (static_cast<unsigned>(frame[1]) << 8) | frame[2];
        current = (static_cast<unsigned>(frame[3]) << 8) | frame[4];
        consumption = (static_cast<unsigned>(frame[5]) << 8) | frame[6];
        erpm = (static_cast<unsigned>(frame[7]) << 8) | frame[8];
    }

    char payload[TX_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload), "ESC,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%.3f,%.3f,%u,%u",
             static_cast<unsigned long>(escTelemetrySequence++),
             static_cast<unsigned long>(millis()),
             static_cast<unsigned>(motor + 1U),
             static_cast<unsigned>(status),
             temperature, voltage, current, consumption, erpm,
             commandToFraction(appliedThrusterCommand[motor]),
             commandToFraction(targetThrusterCommand[motor]),
             static_cast<unsigned>(dshotValue[motor]),
             static_cast<unsigned>(escCurrentAdc));
    sendFramedPayload(payload);
}

// Samples the current pin at its fixed slow rate (see ESC_CURRENT_SAMPLE_INTERVAL_MS).
void updateEscCurrentSample(uint32_t now)
{
    if (static_cast<uint32_t>(now - lastEscCurrentSampleMs) >= ESC_CURRENT_SAMPLE_INTERVAL_MS)
    {
        lastEscCurrentSampleMs = now;
        escCurrentAdc = static_cast<uint16_t>(analogRead(ESC_CURRENT_PIN));
    }
}

// Records the outcome of the last request and tracks since when all four ESCs
// have been answering without a miss.
void noteEscAnswer(size_t motor, bool valid, uint32_t now)
{
    escAnswering[motor] = valid;

    bool all = true;
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        all = all && escAnswering[i];
    }

    if (all && !escAllAnswering)
    {
        escAllAnsweringSinceMs = now;
    }
    escAllAnswering = all;
}

// Fast requests only once every ESC has answered steadily for a while.
uint32_t escTelemetryRequestIntervalMs(uint32_t now)
{
    const bool fast =
        escAllAnswering &&
        static_cast<uint32_t>(now - escAllAnsweringSinceMs) >= ESC_TELEMETRY_FAST_AFTER_MS;
    return fast ? ESC_TELEMETRY_FAST_REQUEST_INTERVAL_MS : ESC_TELEMETRY_SLOW_REQUEST_INTERVAL_MS;
}

// Ask one ESC, wait for its 10 bytes, publish, move on to the next ESC.
// Only reports: it never changes a thruster command.
void updateEscTelemetry()
{
    const uint32_t now = millis();

    updateEscCurrentSample(now);

    // Let the ESCs arm before the first telemetry request.
    if (static_cast<uint32_t>(now - dshotStartMs) < ESC_TELEMETRY_START_DELAY_MS)
    {
        for (size_t i = 0; i < THRUSTER_COUNT; ++i)
        {
            escLastValidMs[i] = now;
        }
        escTelemetryRequestMs = now;
        return;
    }

    bool anyStale = false;
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        if (static_cast<uint32_t>(now - escLastValidMs[i]) > ESC_TELEMETRY_STALE_MS)
        {
            anyStale = true;
        }
    }
    setPersistentError(ERROR_ESC_TELEMETRY_STALE, anyStale);

    if (!escTelemetryWaiting)
    {
        if (static_cast<uint32_t>(now - escTelemetryRequestMs) <
            escTelemetryRequestIntervalMs(now))
        {
            return;
        }

        // Drop anything left over so the next frame starts on a clean buffer.
        while (ESC_TELEMETRY_SERIAL.available() > 0)
        {
            ESC_TELEMETRY_SERIAL.read();
        }

        escTelemetryLength = 0;
        escTelemetryRequestMs = now;
        escTelemetryWaiting = true;
        escTelemetryRequestMotor = static_cast<int8_t>(escTelemetryMotor);
        return;
    }

    while (ESC_TELEMETRY_SERIAL.available() > 0 &&
           escTelemetryLength < ESC_TELEMETRY_FRAME_SIZE)
    {
        escTelemetryBuffer[escTelemetryLength++] =
            static_cast<uint8_t>(ESC_TELEMETRY_SERIAL.read());
    }

    if (escTelemetryLength == ESC_TELEMETRY_FRAME_SIZE)
    {
        if (escTelemetryCrc8(escTelemetryBuffer, ESC_TELEMETRY_FRAME_SIZE - 1U) ==
            escTelemetryBuffer[ESC_TELEMETRY_FRAME_SIZE - 1U])
        {
            escLastValidMs[escTelemetryMotor] = now;
            noteEscAnswer(escTelemetryMotor, true, now);
            publishEscFrame(escTelemetryMotor, ESC_STATUS_VALID, escTelemetryBuffer);
        }
        else
        {
            noteEscAnswer(escTelemetryMotor, false, now);
            publishEscFrame(escTelemetryMotor, ESC_STATUS_CORRUPT, escTelemetryBuffer);
        }
    }
    else if (static_cast<uint32_t>(now - escTelemetryRequestMs) >
             ESC_TELEMETRY_RESPONSE_TIMEOUT_MS)
    {
        noteEscAnswer(escTelemetryMotor, false, now);
        publishEscFrame(escTelemetryMotor, ESC_STATUS_NO_ANSWER, escTelemetryBuffer);
    }
    else
    {
        return;  // still waiting
    }

    escTelemetryWaiting = false;
    escTelemetryMotor = (escTelemetryMotor + 1U) % THRUSTER_COUNT;
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

bool parseUint32(const char* text, uint32_t& value)
{
    if (text == nullptr || *text == '\0')
    {
        return false;
    }

    char* endPtr = nullptr;
    const unsigned long parsed = strtoul(text, &endPtr, 10);
    if (endPtr == text || *endPtr != '\0')
    {
        return false;
    }

    value = static_cast<uint32_t>(parsed);
    return true;
}

bool parseFloat(const char* text, float& value)
{
    if (text == nullptr || *text == '\0')
    {
        return false;
    }

    char* endPtr = nullptr;
    const float parsed = strtof(text, &endPtr);
    if (endPtr == text || *endPtr != '\0' || !isfinite(parsed))
    {
        return false;
    }

    value = parsed;
    return true;
}

// Fraction (1.00 = full throttle) -> thousandths. Input far outside 0..1 is
// limited here; the caller clamps to the configured minimum and maximum.
bool parseFractionToCommand(const char* text, int32_t& command)
{
    float fraction = 0.0f;
    if (!parseFloat(text, fraction))
    {
        return false;
    }

    if (fraction < -10.0f)
    {
        fraction = -10.0f;
    }
    if (fraction > 10.0f)
    {
        fraction = 10.0f;
    }

    command = static_cast<int32_t>(
        lroundf(fraction * static_cast<float>(THRUSTER_COMMAND_FULL_SCALE)));
    return true;
}

void handlePwmCommand(char* savePtr)
{
    uint32_t commandSequence = 0;
    const char* sequenceText = strtok_r(nullptr, ",", &savePtr);
    if (!parseUint32(sequenceText, commandSequence))
    {
        sendError("PWM", 0, "BAD_SEQUENCE");
        return;
    }

    uint16_t requestedPwm[THRUSTER_COUNT];
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        int32_t parsedPwm = 0;
        const char* pwmText = strtok_r(nullptr, ",", &savePtr);
        if (!parseFractionToCommand(pwmText, parsedPwm))
        {
            sendError("PWM", commandSequence, "BAD_PWM");
            return;
        }
        requestedPwm[i] = clampThrusterCommand(parsedPwm);
    }

    // Optional final field: 0 = ROS command source is fresh, 1 = stale.
    // If omitted, assume fresh for backward compatibility.
    uint32_t sourceStale = 0;
    const char* sourceStaleText = strtok_r(nullptr, ",", &savePtr);
    if (sourceStaleText != nullptr)
    {
        if (!parseUint32(sourceStaleText, sourceStale) || sourceStale > 1U)
        {
            sendError("PWM", commandSequence, "BAD_SOURCE_STATUS");
            return;
        }
    }

    if (strtok_r(nullptr, ",", &savePtr) != nullptr)
    {
        sendError("PWM", commandSequence, "TOO_MANY_FIELDS");
        return;
    }

    // A valid frame proves that the Jetson-Teensy serial link is alive.
    lastPwmCommandMs = millis();
    receivedPwmCommand = true;
    rosCommandStale = sourceStale != 0U;

    // During tare, preserve the already-applied thruster command. After tare, the next frame
    // applies the newest request. At no point is PWM forced to zero/startup.
    if (!tareInProgress)
    {
        setThrusterTargets(requestedPwm);
    }
}

void handleTareCommand(char* savePtr)
{
    uint32_t commandSequence = 0;
    const char* sequenceText = strtok_r(nullptr, ",", &savePtr);

    if (!parseUint32(sequenceText, commandSequence) ||
        strtok_r(nullptr, ",", &savePtr) != nullptr)
    {
        sendError("TARE", commandSequence, "BAD_FORMAT");
        return;
    }

    if (tareInProgress)
    {
        sendError("TARE", commandSequence, "BUSY");
        return;
    }

    // Keep all thrusters at their current thruster command while taring.
    if (tareBothLoadCells())
    {
        sendAck("TARE", commandSequence);
    }
    else
    {
        sendError("TARE", commandSequence, "HX711_TIMEOUT");
    }
}

void handleCalibrationCommand(char* savePtr)
{
    uint32_t commandSequence = 0;
    float newFrontFactor = 0.0f;
    float newRearFactor = 0.0f;

    const char* sequenceText = strtok_r(nullptr, ",", &savePtr);
    const char* frontText = strtok_r(nullptr, ",", &savePtr);
    const char* rearText = strtok_r(nullptr, ",", &savePtr);

    if (!parseUint32(sequenceText, commandSequence) ||
        !parseFloat(frontText, newFrontFactor) ||
        !parseFloat(rearText, newRearFactor) ||
        fabsf(newFrontFactor) < 1.0e-9f ||
        fabsf(newRearFactor) < 1.0e-9f ||
        strtok_r(nullptr, ",", &savePtr) != nullptr)
    {
        sendError("CAL", commandSequence, "BAD_FACTORS");
        return;
    }

    frontCountsPerGram = newFrontFactor;
    rearCountsPerGram = newRearFactor;
    sendAck("CAL", commandSequence);
}

void handleConfigurationCommand(char* savePtr)
{
    uint32_t commandSequence = 0;
    int32_t newMin = 0;
    int32_t newMax = 0;
    int32_t newStartup = 0;
    uint32_t newTimeout = 0;
    float rampDownStepFraction = 0.0f;
    uint32_t newRampDownInterval = 0;

    const char* sequenceText = strtok_r(nullptr, ",", &savePtr);
    const char* minText = strtok_r(nullptr, ",", &savePtr);
    const char* maxText = strtok_r(nullptr, ",", &savePtr);
    const char* startupText = strtok_r(nullptr, ",", &savePtr);
    const char* timeoutText = strtok_r(nullptr, ",", &savePtr);
    const char* rampStepText = strtok_r(nullptr, ",", &savePtr);
    const char* rampIntervalText = strtok_r(nullptr, ",", &savePtr);

    // min, max, startup and ramp step are fractions of full throttle.
    if (!parseUint32(sequenceText, commandSequence) ||
        !parseFractionToCommand(minText, newMin) ||
        !parseFractionToCommand(maxText, newMax) ||
        !parseFractionToCommand(startupText, newStartup) ||
        !parseUint32(timeoutText, newTimeout) ||
        !parseFloat(rampStepText, rampDownStepFraction) ||
        !parseUint32(rampIntervalText, newRampDownInterval) ||
        strtok_r(nullptr, ",", &savePtr) != nullptr)
    {
        sendError("CFG", commandSequence, "BAD_CONFIG");
        return;
    }

    const int32_t fullScale = static_cast<int32_t>(THRUSTER_COMMAND_FULL_SCALE);
    const int32_t newRampDownStep =
        rampDownStepFraction >= 0.0f && rampDownStepFraction <= 1.0f
            ? static_cast<int32_t>(lroundf(rampDownStepFraction * static_cast<float>(fullScale)))
            : -1;

    // A step below the 0.001 resolution would round to 0, which means "no ramp".
    // It is rejected so that a very small step can never turn into an instant drop.
    const bool stepBelowResolution = rampDownStepFraction > 0.0f && newRampDownStep == 0;

    if (newMin < 0 || newMax > fullScale || newMin >= newMax ||
        newStartup < newMin || newStartup > newMax ||
        newTimeout < 50U || newTimeout > 10000U ||
        newRampDownStep < 0 || stepBelowResolution ||
        newRampDownInterval < 1U || newRampDownInterval > 10000U)
    {
        sendError("CFG", commandSequence, "BAD_CONFIG");
        return;
    }

    thrusterMinCommand = static_cast<uint16_t>(newMin);
    thrusterMaxCommand = static_cast<uint16_t>(newMax);
    startupThrusterCommand = static_cast<uint16_t>(newStartup);
    commandTimeoutMs = newTimeout;
    rampDownStepCommand = static_cast<uint16_t>(newRampDownStep);
    rampDownIntervalMs = newRampDownInterval;

    if (!receivedPwmCommand)
    {
        applyStartupThrusterCommand();
    }
    else
    {
        // Clamp both the active output and the pending target to the new limits.
        // Do not otherwise alter the current thrust during reconfiguration.
        for (size_t i = 0; i < THRUSTER_COUNT; ++i)
        {
            targetThrusterCommand[i] =
                clampThrusterCommand(targetThrusterCommand[i]);
            writeAppliedThrusterCommand(i, appliedThrusterCommand[i]);

            if (targetThrusterCommand[i] > appliedThrusterCommand[i])
            {
                writeAppliedThrusterCommand(i, targetThrusterCommand[i]);
            }
        }
    }

    sendAck("CFG", commandSequence);
}

void processReceivedLine(char* line)
{
    if (!validateAndStripCrc(line))
    {
        recordTransientError(ERROR_BAD_SERIAL_FRAME);
        return;
    }

    char* savePtr = nullptr;
    const char* command = strtok_r(line, ",", &savePtr);
    if (command == nullptr)
    {
        recordTransientError(ERROR_BAD_COMMAND);
        return;
    }

    if (strcmp(command, "PWM") == 0)
    {
        handlePwmCommand(savePtr);
    }
    else if (strcmp(command, "TARE") == 0)
    {
        handleTareCommand(savePtr);
    }
    else if (strcmp(command, "CAL") == 0)
    {
        handleCalibrationCommand(savePtr);
    }
    else if (strcmp(command, "CFG") == 0)
    {
        handleConfigurationCommand(savePtr);
    }
    else
    {
        sendError("UNKNOWN", 0, "UNKNOWN_COMMAND");
    }
}

void readSerialInput()
{
    while (Serial.available() > 0)
    {
        const char received = static_cast<char>(Serial.read());

        if (received == '\n')
        {
            rxBuffer[rxLength] = '\0';
            if (rxLength > 0)
            {
                processReceivedLine(rxBuffer);
            }
            rxLength = 0;
        }
        else if (received != '\r')
        {
            if (rxLength < RX_BUFFER_SIZE - 1U)
            {
                rxBuffer[rxLength++] = received;
            }
            else
            {
                rxLength = 0;
                recordTransientError(ERROR_RX_BUFFER_OVERFLOW);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Arduino lifecycle
// -----------------------------------------------------------------------------

void setup()
{
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.begin(115200);

    if (ENABLE_LOAD_CELLS)
    {
        frontLoadCell.begin(FRONT_HX711_DAT_PIN, FRONT_HX711_SCK_PIN);
        rearLoadCell.begin(REAR_HX711_DAT_PIN, REAR_HX711_SCK_PIN);
    }

    // ESC current pin as a plain high-impedance analog input.
    pinMode(ESC_CURRENT_PIN, INPUT_DISABLE);
    analogReadResolution(ADC_RESOLUTION_BITS);
    analogReadAveraging(ADC_HARDWARE_AVERAGING);

    ESC_TELEMETRY_SERIAL.begin(ESC_TELEMETRY_BAUD);

    // Start the DShot frames at the startup command so the ESCs see a valid
    // signal from the first millisecond.
    for (size_t i = 0; i < THRUSTER_COUNT; ++i)
    {
        targetThrusterCommand[i] = startupThrusterCommand;
        appliedThrusterCommand[i] = startupThrusterCommand;
        dshotValue[i] = thrusterCommandToDshot(startupThrusterCommand);
        escLastValidMs[i] = millis();
    }
    beginDshot();

    applyStartupThrusterCommand();
    lastRampDownUpdateMs = millis();
    responsiveDelay(ESC_STARTUP_DELAY_MS);

    const uint32_t now = millis();
    lastFrontReadyMs = now;
    lastRearReadyMs = now;

    if (AUTO_TARE_ON_STARTUP)
    {
        tareBothLoadCells();
    }

    // Give the ROS bridge one command-timeout period to connect before showing
    // a serial communication error.
    lastPwmCommandMs = millis();

    sendFramedPayload("STATUS,BOOT,OK");
    escFramesEnabled = true;
}

void loop()
{
    readSerialInput();
    updateThrusterOutputs();
    updateCommunicationHealth();
    updateHx711Health();
    publishSensorDataIfReady();
    updateEscTelemetry();
    updateStatusLed();
}
