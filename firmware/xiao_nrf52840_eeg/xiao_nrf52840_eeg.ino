// =============================================================================
//  XIAO nRF52840  –  8-Channel EEG via ADS1299  –  BLE 250 SPS
// =============================================================================
//
//  PACKET FORMAT (27 bytes per sample)
//  ┌────────┬──────────┬────────────────────────────────────────────┬────────┐
//  │ 0xA0   │ seq (1B) │ CH1-CH8 each 3 bytes MSB-first (24 bytes) │ 0xC0   │
//  └────────┴──────────┴────────────────────────────────────────────┴────────┘
//  Total  = 1 + 1 + 24 + 1 = 27 bytes
//
//  MTU BATCHING
//  At connection the web browser (Chrome) requests a large ATT MTU (up to 512).
//  The nRF52840 SoftDevice caps it at 247 → 244 bytes payload.
//  244 / 27 = 9 complete samples per BLE notification.
//  At 250 SPS this gives ~27 notifications/second (one every ~36 ms).
//  If MTU is smaller the code falls back to fewer samples per packet.
//
//  WIRING  (XIAO nRF52840 → ADS1299)
//  ───────────────────────────────────
//  D8  (SCK)   → SCLK
//  D9  (MISO)  → DOUT
//  D10 (MOSI)  → DIN
//  D7          → CS   (chip-select, active LOW)
//  D5          → START
//  D4          → RESET (active LOW)
//  D3          → DRDY  (active LOW, fires interrupt)
//  3.3V        → AVDD, DVDD (use 3.3V NOT 5V)
//  GND         → AGND, DGND
//
//  REQUIRED LIBRARIES (install via Arduino Library Manager)
//  ─────────────────────────────────────────────────────────
//  - ArduinoBLE  (>= 1.3.0)
//  Board package: "Seeed nRF52 mbed-enabled Boards"  OR
//                 "Seeed XIAO nRF52840" from Seeed board manager URL
//
//  BLE UUIDs match the web app exactly so no web-side UUID changes needed.
// =============================================================================

#include <ArduinoBLE.h>
#include <SPI.h>

// ── BLE UUIDs (must match web app) ──────────────────────────────────────────
#define DEVICE_NAME      "EEGlasses"
#define SERVICE_UUID     "0000a000-0000-1000-8000-00805f9b34fb"
#define CHAR_READ_UUID   "0000a001-0000-1000-8000-00805f9b34fb"
#define CHAR_WRITE_UUID  "0000a002-0000-1000-8000-00805f9b34fb"
#define CHAR_DATA_UUID   "0000a003-0000-1000-8000-00805f9b34fb"

// ── ADS1299 pin assignments ──────────────────────────────────────────────────
#define PIN_ADS_CS     D7
#define PIN_ADS_START  D5
#define PIN_ADS_RESET  D4
#define PIN_ADS_DRDY   D3

// ── Packet / buffer constants ────────────────────────────────────────────────
#define START_BYTE        0xA0
#define END_BYTE          0xC0
#define BYTES_PER_SAMPLE  27          // 1+1+24+1
#define MAX_PAYLOAD       244         // nRF52840 SoftDevice max (MTU 247 - 3)
#define MAX_SAMPLES_PKT   (MAX_PAYLOAD / BYTES_PER_SAMPLE)   // 9
#define TX_BUF_SIZE       (MAX_SAMPLES_PKT * BYTES_PER_SAMPLE)  // 243 bytes

// ── ADS1299 register addresses ───────────────────────────────────────────────
#define REG_CONFIG1  0x01
#define REG_CONFIG2  0x02
#define REG_CONFIG3  0x03
#define REG_CH1SET   0x05
#define REG_CH8SET   0x0C

// ── ADS1299 SPI commands ─────────────────────────────────────────────────────
#define CMD_SDATAC   0x11   // stop continuous data read
#define CMD_RDATAC   0x10   // start continuous data read
#define CMD_START    0x08   // start/restart conversions
#define CMD_RESET    0x06   // reset (software)
#define CMD_RREG     0x20   // read register(s)
#define CMD_WREG     0x40   // write register(s)

// ── BLE objects ──────────────────────────────────────────────────────────────
BLEService        eegService  (SERVICE_UUID);
BLECharacteristic cmdReadChar (CHAR_READ_UUID,  BLERead | BLENotify, 20);
BLECharacteristic cmdWriteChar(CHAR_WRITE_UUID, BLEWrite, 20);
BLECharacteristic eegDataChar (CHAR_DATA_UUID,  BLERead | BLENotify, TX_BUF_SIZE);

// ── Runtime state ─────────────────────────────────────────────────────────────
static uint8_t  txBuf[TX_BUF_SIZE];
static uint8_t  sampleSeq    = 0;
static int      samplesInBuf = 0;
static volatile bool drdyReady = false;

// =============================================================================
//  ADS1299 Low-level SPI helpers
// =============================================================================

static void adsSelect()   { digitalWrite(PIN_ADS_CS, LOW);  delayMicroseconds(1); }
static void adsDeselect() { digitalWrite(PIN_ADS_CS, HIGH); delayMicroseconds(2); }

static void adsCmd(uint8_t cmd) {
    adsSelect();
    SPI.transfer(cmd);
    adsDeselect();
}

static void adsWriteReg(uint8_t reg, uint8_t val) {
    adsSelect();
    SPI.transfer(CMD_WREG | (reg & 0x1F)); // opcode 1: WREG | address
    SPI.transfer(0x00);                    // opcode 2: write 1 register
    SPI.transfer(val);
    adsDeselect();
}

static uint8_t adsReadReg(uint8_t reg) {
    adsSelect();
    SPI.transfer(CMD_RREG | (reg & 0x1F));
    SPI.transfer(0x00);
    uint8_t val = SPI.transfer(0x00);
    adsDeselect();
    return val;
}

// Read one frame: 3 status bytes + 8 × 3 channel bytes = 27 bytes
// Returns only the 24 channel bytes in chData[24]
static void adsReadFrame(uint8_t chData[24]) {
    uint8_t raw[27];
    adsSelect();
    for (int i = 0; i < 27; i++) {
        raw[i] = SPI.transfer(0x00);
    }
    adsDeselect();
    memcpy(chData, raw + 3, 24); // skip 3 status bytes
}

// =============================================================================
//  ADS1299 Initialisation
// =============================================================================

static void adsInit() {
    // Configure control pins
    pinMode(PIN_ADS_CS,    OUTPUT); digitalWrite(PIN_ADS_CS,    HIGH);
    pinMode(PIN_ADS_START, OUTPUT); digitalWrite(PIN_ADS_START, LOW);
    pinMode(PIN_ADS_RESET, OUTPUT); digitalWrite(PIN_ADS_RESET, HIGH);
    pinMode(PIN_ADS_DRDY,  INPUT);

    // SPI: ADS1299 uses Mode 1 (CPOL=0, CPHA=1), max 20 MHz but 4 MHz is safe
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

    // Hardware reset
    delay(100);
    digitalWrite(PIN_ADS_RESET, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_ADS_RESET, HIGH);
    delay(10);

    adsCmd(CMD_SDATAC);   // must stop continuous read before writing registers
    delay(1);

    // CONFIG1: 250 SPS
    //   Bit7=1 (DAISY_EN disabled=standalone), Bit6=0 (CLK out off)
    //   Bit5..3=001 (reserved default), Bit2..0=110 (DR=250 SPS)
    adsWriteReg(REG_CONFIG1, 0x96);

    // CONFIG2: test signal off, internal reference, default
    adsWriteReg(REG_CONFIG2, 0xC0);

    // CONFIG3: internal reference enabled (VREF=4.5V), bias buffer on
    //   Bit7=1 (PD_REFBUF on), Bit6=1 (reserved), Bit5=1 (BIAS_MEAS off),
    //   Bit4..3=10 (BIASREF int), Bit2=1 (PD_BIAS on), Bit1..0=00
    adsWriteReg(REG_CONFIG3, 0xEC);
    delay(10); // wait for internal reference to settle

    // All 8 channel registers: Gain=24, normal electrode input
    //   CHnSET: Bit6-4=110 (GAIN=24), Bit2-0=000 (normal input)
    for (uint8_t reg = REG_CH1SET; reg <= REG_CH8SET; reg++) {
        adsWriteReg(reg, 0x60);
    }

    // Attach DRDY interrupt – falling edge signals new data ready
    attachInterrupt(digitalPinToInterrupt(PIN_ADS_DRDY), []{ drdyReady = true; }, FALLING);

    // Start conversions and enter continuous-read mode
    adsCmd(CMD_START);
    delay(1);
    adsCmd(CMD_RDATAC);
}

// =============================================================================
//  Packet builder – append one sample to txBuf
// =============================================================================

static void appendSample(const uint8_t chData[24]) {
    uint8_t *p = txBuf + samplesInBuf * BYTES_PER_SAMPLE;
    p[0] = START_BYTE;
    p[1] = sampleSeq++;
    memcpy(p + 2, chData, 24);
    p[26] = END_BYTE;
    samplesInBuf++;
}

// =============================================================================
//  Determine how many samples fit in one notification given current MTU
// =============================================================================

static int samplesPerPacket(BLEDevice &central) {
    int mtu = central.mtu();
    if (mtu <= 0 || mtu < 30) mtu = 247;   // default to max if not reported
    int payload = mtu - 3;                  // ATT overhead = 3 bytes
    int n = payload / BYTES_PER_SAMPLE;
    if (n < 1)  n = 1;
    if (n > MAX_SAMPLES_PKT) n = MAX_SAMPLES_PKT;
    return n;
}

// =============================================================================
//  setup / loop
// =============================================================================

void setup() {
    Serial.begin(115200);

    // Initialise BLE
    if (!BLE.begin()) {
        Serial.println("[ERROR] BLE init failed – halting");
        while (1) {}
    }

    BLE.setLocalName(DEVICE_NAME);
    BLE.setDeviceName(DEVICE_NAME);

    eegService.addCharacteristic(cmdReadChar);
    eegService.addCharacteristic(cmdWriteChar);
    eegService.addCharacteristic(eegDataChar);

    BLE.addService(eegService);
    BLE.setAdvertisedService(eegService);
    BLE.advertise();

    Serial.print("[BLE] Advertising as: ");
    Serial.println(DEVICE_NAME);
    Serial.print("[BLE] Service UUID : ");
    Serial.println(SERVICE_UUID);
    Serial.print("[Packet] Bytes/sample: ");
    Serial.print(BYTES_PER_SAMPLE);
    Serial.print("  Max samples/pkt: ");
    Serial.println(MAX_SAMPLES_PKT);

    adsInit();
    Serial.println("[ADS1299] Initialised at 250 SPS, Gain=24");
}

void loop() {
    BLEDevice central = BLE.central();

    if (central && central.connected()) {
        Serial.print("[BLE] Connected: ");
        Serial.println(central.address());

        int spPkt = samplesPerPacket(central);
        Serial.print("[BLE] Samples per packet: ");
        Serial.println(spPkt);

        samplesInBuf = 0;
        sampleSeq    = 0;

        while (central.connected()) {
            BLE.poll();

            // ── New ADS1299 sample available ──────────────────────────────
            if (drdyReady) {
                drdyReady = false;

                uint8_t chData[24];
                adsReadFrame(chData);
                appendSample(chData);

                if (samplesInBuf >= spPkt) {
                    eegDataChar.writeValue(txBuf, samplesInBuf * BYTES_PER_SAMPLE);
                    samplesInBuf = 0;
                    // Re-query MTU in case it changed (rare, but safe)
                    spPkt = samplesPerPacket(central);
                }
            }

            // ── Handle command from web app ───────────────────────────────
            if (cmdWriteChar.written()) {
                uint8_t cmd = cmdWriteChar.value()[0];
                Serial.print("[CMD] Received: 0x");
                Serial.println(cmd, HEX);

                if (cmd == 0x99) {
                    // Status reply: 0x01 = streaming OK
                    uint8_t reply[1] = { 0x01 };
                    cmdReadChar.writeValue(reply, 1);
                }
            }
        }

        Serial.println("[BLE] Central disconnected – restarting advertisement");
        samplesInBuf = 0;
        BLE.advertise();
    }
}
