// =============================================================================
//  XIAO nRF52840 – 8-Channel SIMULATED EEG – Bluefruit library – 250 SPS
// =============================================================================
//
//  No real hardware needed.  Generates realistic synthetic EEG waveforms
//  (delta / theta / alpha / beta + noise) on all 8 channels and sends them
//  over BLE at 250 SPS, batching multiple samples per notification based on
//  the negotiated ATT MTU.
//
//  PACKET FORMAT  (9 bytes per sample – fits default BLE MTU=23 / payload=20)
//  ┌────────┬──────────┬───────────────────────┬────────┐
//  │  0xA0  │ seq (1B) │ CH1-CH2, 3B each MSB  │  0xC0  │
//  └────────┴──────────┴───────────────────────┴────────┘
//
//  MTU BATCHING
//  20-byte payload / 9 bytes = 2 samples per notification → 125 notif/s @ 250 SPS.
//  Works with Windows default MTU=23 – no MTU negotiation needed.
//
//  BLE UUIDs match the web app exactly – no web changes needed.
//
//  BOARD SETUP
//  ──────────────────────────────────────────────────────────────────────
//  Board manager URL (Seeed):
//    https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
//  Select board: Seeed XIAO nRF52840
//  The Bluefruit (Adafruit nRF52) library comes bundled with the Seeed nRF52
//  board package, or install separately:
//    Library Manager → "Adafruit Bluefruit nRF52"
// =============================================================================

#include <bluefruit.h>

// ── BLE identifiers ──────────────────────────────────────────────────────────
#define DEVICE_NAME     "hiibrarahmad-EEG"
#define SERVICE_UUID    "0000a000-0000-1000-8000-00805f9b34fb"
#define CHAR_READ_UUID  "0000a001-0000-1000-8000-00805f9b34fb"
#define CHAR_WRITE_UUID "0000a002-0000-1000-8000-00805f9b34fb"
#define CHAR_DATA_UUID  "0000a003-0000-1000-8000-00805f9b34fb"

// ── Packet sizing ────────────────────────────────────────────────────────────
#define START_BYTE        0xA0
#define END_BYTE          0xC0
#define BYTES_PER_SAMPLE  9             // [0xA0][seq][CH1 3B][CH2 3B][0xC0]
#define NUM_CHANNELS      2             // 2 channels sent per sample
#define MAX_PAYLOAD       20            // worst-case: MTU=23 → 20B payload
#define MAX_SAMPLES_PKT   (MAX_PAYLOAD / BYTES_PER_SAMPLE)   // = 2
#define TX_BUF_SIZE       (MAX_SAMPLES_PKT * BYTES_PER_SAMPLE)  // = 18

// ── Sample timing ────────────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ      250
#define SAMPLE_PERIOD_MS    (1000 / SAMPLE_RATE_HZ)   // = 4 ms

// ── BLE objects ───────────────────────────────────────────────────────────────
BLEService        eegService  (SERVICE_UUID);
BLECharacteristic cmdReadChar (CHAR_READ_UUID);
BLECharacteristic cmdWriteChar(CHAR_WRITE_UUID);
BLECharacteristic eegDataChar (CHAR_DATA_UUID);

// ── State ────────────────────────────────────────────────────────────────────
static uint8_t  txBuf[TX_BUF_SIZE];
static uint8_t  sampleSeq        = 0;
static int      samplesInBuf     = 0;
static uint16_t activeConnHandle = BLE_CONN_HANDLE_INVALID;static bool     streamingStarted = false;   // true once MTU is large enough
SoftwareTimer sampleTimer;
volatile bool sampleReady = false;

// =============================================================================
//  Lightweight LCG pseudo-random noise (no stdlib rand overhead)
// =============================================================================
static uint32_t lcgState = 0xDEADBEEFUL;
static inline int32_t lcgNoise(int32_t amp) {
    lcgState = lcgState * 1664525UL + 1013904223UL;
    int32_t r = (int32_t)(lcgState >> 16) & 0xFFFF;
    return (r - 32768) * amp / 32768;
}

// =============================================================================
//  Synthetic EEG – per-channel waveform generator
//  Returns a 24-bit signed integer (clamped to ±8 388 607).
//  Each channel has unique phase offsets and amplitude ratios so the 8
//  channels look independent but all plausibly "brain-like".
// =============================================================================

// Channel-specific phase offsets (radians)
static const float CH_PHASE[8]        = { 0.00f, 0.52f, 1.05f, 1.57f,
                                          2.09f, 2.62f, 3.14f, 3.67f };
// Small alpha-frequency offset per channel (Hz)
static const float CH_ALPHA_DELTA[8]  = { 0.0f,  0.3f, -0.3f,  0.6f,
                                         -0.6f,  0.9f, -0.9f,  1.2f };

static int32_t syntheticEEG(uint8_t ch, float t) {
    float phi = CH_PHASE[ch];

    // Delta  1-4 Hz  – large slow waves
    float delta = 120000.f * sinf(TWO_PI * (1.8f + ch * 0.28f) * t + phi);

    // Theta  4-8 Hz  – drowsiness / creativity
    float theta =  80000.f * sinf(TWO_PI * (5.5f + ch * 0.18f) * t + phi + 0.5f);

    // Alpha  8-13 Hz – dominant "eyes closed" rhythm
    float alpha = 220000.f * sinf(TWO_PI * (10.f + CH_ALPHA_DELTA[ch]) * t + phi + 1.1f);

    // Beta  13-30 Hz – active cognition, smaller amplitude
    float beta  =  45000.f * sinf(TWO_PI * (20.f + ch * 0.9f)  * t + phi + 2.1f);

    // Gamma burst on ch1 only (40 Hz, tiny)
    float gamma = (ch == 0) ? 12000.f * sinf(TWO_PI * 40.f * t) : 0.f;

    // Noise (sum two LCG calls ≈ triangular distribution)
    int32_t noise = lcgNoise(6000) + lcgNoise(6000);

    int32_t v = (int32_t)(delta + theta + alpha + beta + gamma) + noise;
    if (v >  8388607)  v =  8388607;
    if (v < -8388608)  v = -8388608;
    return v;
}

// =============================================================================
//  Build one sample frame into txBuf at current write position
// =============================================================================
static void appendSample(float t) {
    uint8_t *p = txBuf + samplesInBuf * BYTES_PER_SAMPLE;
    p[0] = START_BYTE;
    p[1] = sampleSeq++;

    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        int32_t  raw  = syntheticEEG(ch, t);
        uint32_t u24  = (uint32_t)(raw & 0xFFFFFF);
        p[2 + ch * 3]     = (u24 >> 16) & 0xFF;
        p[2 + ch * 3 + 1] = (u24 >>  8) & 0xFF;
        p[2 + ch * 3 + 2] =  u24        & 0xFF;
    }

    p[BYTES_PER_SAMPLE - 1] = END_BYTE;   // byte 8
    samplesInBuf++;
}

// =============================================================================
//  Timer callback – sets a flag every 4 ms (250 Hz)
// =============================================================================
void sampleTimerCB(TimerHandle_t xTimer) {
    (void)xTimer;
    sampleReady = true;
}

// =============================================================================
//  TX diagnostics counters (file-scope so connectCB and loop() can both access)
// =============================================================================
static uint32_t _lastDiag = 0;
static uint32_t _txCount  = 0;
static uint32_t _txFail   = 0;

// =============================================================================
//  BLE connect / disconnect callbacks
// =============================================================================
void connectCB(uint16_t conn_handle) {
    activeConnHandle = conn_handle;
    streamingStarted = false;
    samplesInBuf     = 0;
    _txCount = 0;  _txFail = 0;

    BLEConnection *conn = Bluefruit.Connection(conn_handle);
    conn->requestConnectionParameter(6, 24); // 7.5 ms – 30 ms interval

    char name[64] = {};
    conn->getPeerName(name, sizeof(name));
    Serial.print("[BLE] Connected to: ");
    Serial.println(name[0] ? name : "(unnamed)");
    Serial.print("[BLE] MTU: "); Serial.print(conn->getMtu());
    Serial.println(" (need >= 12 for 9B frames)");
    // ⚠ sampleTimer starts in loop() once MTU check passes (>= 12)
}

void disconnectCB(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    activeConnHandle = BLE_CONN_HANDLE_INVALID;
    streamingStarted = false;
    sampleTimer.stop();
    samplesInBuf = 0;
    Serial.print("[BLE] Disconnected – reason 0x");
    Serial.println(reason, HEX);
    // Advertising restarts automatically (restartOnDisconnect = true)
}

// =============================================================================
//  Command write handler (web → device)
// =============================================================================
void cmdWriteCB(uint16_t conn_handle, BLECharacteristic *chr,
                uint8_t *data, uint16_t len) {
    if (len == 0) return;
    Serial.print("[CMD] Received: 0x");
    Serial.println(data[0], HEX);
    if (data[0] == 0x99) {
        uint8_t reply[1] = { 0x01 };  // streaming OK
        cmdReadChar.notify(conn_handle, reply, 1);
    }
}

// =============================================================================
//  setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println("=== XIAO nRF52840 Simulated EEG Node ===");

    // ── Bluefruit core ────────────────────────────────────────────────────────
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName(DEVICE_NAME);
    Bluefruit.Periph.setConnectCallback(connectCB);
    Bluefruit.Periph.setDisconnectCallback(disconnectCB);

    // ── GATT ─────────────────────────────────────────────────────────────────
    eegService.begin();

    // cmdRead – readable + notifiable, 1 byte
    cmdReadChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    cmdReadChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    cmdReadChar.setFixedLen(1);
    cmdReadChar.begin();

    // cmdWrite – writable (no response preferred for lowest latency), 1 byte
    cmdWriteChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    cmdWriteChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    cmdWriteChar.setFixedLen(1);
    cmdWriteChar.setWriteCallback(cmdWriteCB);
    cmdWriteChar.begin();

    // eegData – notifiable, variable up to TX_BUF_SIZE
    eegDataChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    eegDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    eegDataChar.setMaxLen(TX_BUF_SIZE);
    eegDataChar.begin();

    // ── Advertising ──────────────────────────────────────────────────────────
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(eegService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);  // auto-restart after disconnect
    Bluefruit.Advertising.setInterval(32, 32);        // 20 ms constant – fast discovery
    Bluefruit.Advertising.setFastTimeout(0);          // stay in fast mode indefinitely
    Bluefruit.Advertising.start(0);                   // advertise until connected

    // ── 250 Hz software timer – started only when a central connects ─────────
    // SoftwareTimer::begin(period_ms, callback, oneshot)
    sampleTimer.begin(SAMPLE_PERIOD_MS, sampleTimerCB);

    Serial.println("[BLE] Advertising as: " DEVICE_NAME);
    Serial.print("[BLE] Service: ");    Serial.println(SERVICE_UUID);
    Serial.print("[Packet] ");
    Serial.print(BYTES_PER_SAMPLE);    Serial.print(" bytes/sample (");
    Serial.print(NUM_CHANNELS);        Serial.print(" ch), ");
    Serial.print(MAX_SAMPLES_PKT);     Serial.println(" samples/pkt max @ MTU=23");
    Serial.println("[EEG] CH1+CH2 synthetic: delta+theta+alpha+beta+noise @ 250 SPS");
}

// =============================================================================
//  loop
// =============================================================================
void loop() {
    // Nothing to do until a central is connected
    if (activeConnHandle == BLE_CONN_HANDLE_INVALID) {
        delay(10);
        return;
    }

    BLEConnection *conn = Bluefruit.Connection(activeConnHandle);
    if (!conn || !conn->connected()) {
        samplesInBuf     = 0;
        streamingStarted = false;
        return;
    }

    uint16_t mtu = conn->getMtu();

    // ── MTU check: 9B frame needs MTU >= 12; default MTU=23 always passes ───
    if (!streamingStarted) {
        if (mtu >= (BYTES_PER_SAMPLE + 3)) {          // MTU ≥ 12 – always true at default
            streamingStarted = true;
            sampleTimer.start();
            uint16_t payload = mtu - 3;
            Serial.print("[BLE] Streaming started! MTU="); Serial.print(mtu);
            Serial.print(" payload="); Serial.print(payload);
            Serial.print("B → "); Serial.print(payload / BYTES_PER_SAMPLE);
            Serial.println(" smp/pkt");
        }
        return;
    }

    // ── Streaming phase: wait for 250 Hz timer tick ────────────────────────
    if (!sampleReady) return;
    sampleReady = false;

    // Generate and buffer one sample
    float t = (float)micros() * 1e-6f;
    appendSample(t);

    // How many samples fit in one notification given current MTU?
    uint16_t payload = (mtu > 3) ? (uint16_t)(mtu - 3) : 20;
    int spPkt        = payload / BYTES_PER_SAMPLE;
    if (spPkt < 1) spPkt = 1;
    if (spPkt > MAX_SAMPLES_PKT) spPkt = MAX_SAMPLES_PKT;

    // Flush buffer when full
    if (samplesInBuf >= spPkt) {
        uint16_t txLen = (uint16_t)(samplesInBuf * BYTES_PER_SAMPLE);
        bool ok = eegDataChar.notify(activeConnHandle, txBuf, txLen);
        if (!ok) {
            _txFail++;
            // BLE congestion: drop oldest half to avoid accumulating lag
            samplesInBuf = samplesInBuf / 2;
        } else {
            _txCount++;
            samplesInBuf = 0;
        }
    }

    // Print diagnostics every 5 seconds
    uint32_t now = millis();
    if (now - _lastDiag >= 5000) {
        _lastDiag = now;
        BLEConnection *c = Bluefruit.Connection(activeConnHandle);
        uint16_t curMtu = c ? c->getMtu() : 0;
        Serial.print("[DIAG] MTU="); Serial.print(curMtu);
        Serial.print(" spPkt="); Serial.print(spPkt);
        Serial.print(" TX="); Serial.print(_txCount);
        Serial.print(" FAIL="); Serial.print(_txFail);
        Serial.print(" payload="); Serial.print(curMtu > 3 ? curMtu - 3 : 0);
        Serial.println("B");
    }
}
