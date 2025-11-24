/***********************************************************************************************************
This example Arduino sketch was developed to work with Anabit's open source RangeRite ADC reference design and 
goes with Anabit's tutorial video on using two point calibration method on ADC circuits. Refer to the link below 
to access the video on Anabit's YouTube Channel. 
The RangeRite ADC gets its name from the fact that it supports 9 different voltage ranges: 5 bipolar and 4 unipolar
all generated from a single input power source between 6V and 18V. The RangeRite ADC comes in two resolution 
versions 16 and 18 bit versions. It also comes in two sample rate versions: 100kSPS and 500kSPS. This example
sketch shows you how to set the RangeRite's input votlage range and make a single ADC measurement every 2 sec.
Be sure to look at the initial settings for this sketch including SPI chip select pin, RangeRite resolution
(16 or 18), voltage range, and if you want to use the reset pin. This sketch also allows you to average multiple
readings together and allows you to enter data for a two point calibration to greatly improve the ADC's accuracy.

RangeRite ADC Dev Board Link: https://anabit.co/products/rangerite-18-bit-adc

Two point calibration tutorial video link: 

This example sketch demonstrates how to set the input voltage range and make a single ADC reading
From Texas Instruments ADS868x 16 bit ADC IC family or the ADS869x 18 bit ADC IC family.

Please report any issue with the sketch to the Anabit forum: https://anabit.co/community/forum/analog-to-digital-converters-adcs

Example code developed by Your Anabit LLC © 2025
Licensed under the Apache License, Version 2.0.
*************************************************************************************************************/
#include <Arduino.h>
#include <SPI.h>

// ================== Device selection ==================
#define ADSX_BITS 16               // <-- set to 16 for ADS868x, 18 for ADS869x (default)
#if (ADSX_BITS != 16) && (ADSX_BITS != 18)
#error "ADSX_BITS must be 16 or 18"
#endif

// Conservative conversion time wait (no RVS pin used).
// Safe for ADS8699 (100 kSPS) and faster parts; adjust lower if you want max throughput.
#define ADSX_TCONV_US 12

// Code alignment in 32-bit output word, needed to properly convert the ADC output code measurement
#if ADSX_BITS == 18
  #define ADSX_CODE_SHIFT   14
  #define ADSX_CODE_MASK    0x3FFFFu
#else
  #define ADSX_CODE_SHIFT   16
  #define ADSX_CODE_MASK    0xFFFFu
#endif

// ================== Board pins ==================
static const int PIN_CS  = 10;   // tie ADS86xx CONVST/CS here
static const int PIN_RST = 9;    //Optional reset from MCU e.g., D9; set -1 if not used

// setup SPI communication and clock rate. 
//Make sure the clock rate is supported by the Arduino board you are using
SPISettings adsSPI(20000000, MSBFIRST, SPI_MODE0);

// ================== ADS86xx register/fields ==================
//See section 7.6 Register Maps page 48 in ADS868x and ADS869x datasheet
//register code to select ADC input range
#define ADSX_REG_RANGE_SEL   0x14 
// INTREF control bit (bit 6 in RANGE_SEL low byte)
#define ADSX_INTREF_ENABLE   0x0000  // INTREF_DIS = 0
#define ADSX_INTREF_DISABLE  0x0040  // INTREF_DIS = 1
// RANGE_SEL[3:0] codes (TI table)
#define ADSX_RANGE_BIPOLAR_3X       0x0   // ±3.000 × VREF = +/- 12.288V
#define ADSX_RANGE_BIPOLAR_2P5X     0x1   // ±2.500 × VREF = +/- 10.24V
#define ADSX_RANGE_BIPOLAR_1P5X     0x2   // ±1.500 × VREF = +/- 6.144V
#define ADSX_RANGE_BIPOLAR_1P25X    0x3   // ±1.250 × VREF = +/- 5.12V
#define ADSX_RANGE_BIPOLAR_0P625X   0x4   // ±0.625 × VREF = +/- 2.56V
#define ADSX_RANGE_UNIPOLAR_3X      0x8   // 3.000 × VREF = 12.288V
#define ADSX_RANGE_UNIPOLAR_2P5X    0x9   // 2.500 × VREF = 10.24V
#define ADSX_RANGE_UNIPOLAR_1P5X    0xA   // 1.500 × VREF = 6.144V
#define ADSX_RANGE_UNIPOLAR_1P25X   0xB   // 1.250 × VREF = 5.12V

// ================== Conversion context ==================
static uint16_t g_range_code     = ADSX_RANGE_BIPOLAR_3X; // default 0..1.25×VREF
static bool     g_useInternalRef = true;
static float    g_vref_volts     = 4.096f;   // internal REF (REFCAP) when enabled

// ================== Optional two-point calibration ==================
// Set to 1 to enable two-point calibration, 0 to disable (use raw transfer function only).
#define ADSX_ENABLE_TWO_POINT_CAL  0   // <--- change to 1 to turn calibration ON

// Input range *used when you collected the calibration data*.
// This lets you keep calibration tied to a specific ADSX_RANGE_* value.
static const uint16_t ADSX_CAL_RANGE_CODE = ADSX_RANGE_BIPOLAR_3X;
// Number of samples to average for each reported reading.
// User may change this to any integer ≥ 1.
static uint16_t g_num_averages = 12;   // Default averaging count

// User-supplied calibration points:
//  - ACTUAL   = true input voltage from a trusted reference (DMM, calibrator, etc.).
//  - MEASURED = voltage reported by this sketch *before* calibration, using adsxCodeToVolts()
//               with ADSX_CAL_RANGE_CODE and the same VREF you used during calibration.
static const float ADSX_CAL_P1_ACTUAL   = -10.0002f;
static const float ADSX_CAL_P1_MEASURED = -10.0017f;

static const float ADSX_CAL_P2_ACTUAL   = 10.0027f;
static const float ADSX_CAL_P2_MEASURED = 10.0045f;

// Computed in setup()
static bool  g_cal_enabled = false;
static float g_cal_slope   = 1.0f;
static float g_cal_offset  = 0.0f;

// ================================================================
// Average N ADC conversions and return the averaged voltage value.
// Applies calibration automatically (if enabled).
// ================================================================
float adsxReadAveragedVolts() {
  uint32_t code_accumulator = 0;

  // Ensure we average at least once
  uint16_t n = (g_num_averages < 1) ? 1 : g_num_averages;

  for (uint16_t i = 0; i < n; i++) {
    uint32_t code = adsxReadCodeNbits();
    code_accumulator += code;
    delay(1);
  }

  // Average the ADC code
  float avg_code = (float)code_accumulator / (float)n;

  // Convert raw averaged code to volts
  float volts_raw = adsxCodeToVolts((uint32_t)avg_code);

  // Optionally apply two-point calibration
#if ADSX_ENABLE_TWO_POINT_CAL
  if (g_cal_enabled && (g_range_code == ADSX_CAL_RANGE_CODE)) {
    return g_cal_offset + g_cal_slope * volts_raw;
  }
#endif

  return volts_raw;
}

//SPI helper function to send and read 32 bit SPI package
//input arguments are 4x uint8_t that together equal 32 bit word to send to ADC
//returns 32 bit word read from ADC
static inline uint32_t xfer32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  digitalWrite(PIN_CS, LOW); 
  uint8_t r0 = SPI.transfer(b0);
  uint8_t r1 = SPI.transfer(b1);
  uint8_t r2 = SPI.transfer(b2);
  uint8_t r3 = SPI.transfer(b3);
  digitalWrite(PIN_CS, HIGH);
  return ( (uint32_t)r0<<24 ) | ( (uint32_t)r1<<16 ) | ( (uint32_t)r2<<8 ) | r3;
}

//NOP write data function to send to ADC, reads back uint32_t value from ADC
static inline uint32_t frameNOP() {
  return xfer32(0x00,0x00,0x00,0x00);
}

// fucntion that Writes 16 bits to a register (WRITE HWORD opcode 0xD0)
//first input argument is register address and second is 16 bits of data
static void writeRegHWord(uint8_t addr, uint16_t data) {
  xfer32(0xD0, addr, (uint8_t)(data >> 8), (uint8_t)(data & 0xFF));
  // CS rising edge starts a conversion; wait for conversion to finish (no RVS used).
  delayMicroseconds(ADSX_TCONV_US);
}

//Sets voltage reference used by ADC
void adsxSetVrefVolts(float vref_volts) {
  if (vref_volts > 0.0f) g_vref_volts = vref_volts;
}

//Sets input voltage range of the ADC
//First input argument is voltage range you want to set ADC to
//Second input argument is bool to define whether internal ref is being used
void adsxSetRange(uint16_t range_code, bool useInternalRef = true) {
  uint16_t data = (range_code & 0x000F) |
                  (useInternalRef ? ADSX_INTREF_ENABLE : ADSX_INTREF_DISABLE);

  SPI.beginTransaction(adsSPI);
  writeRegHWord(ADSX_REG_RANGE_SEL, data);
  SPI.endTransaction();

  g_range_code     = (range_code & 0x000F);
  g_useInternalRef = useInternalRef;

  // First conversion after a config write is not valid; optional dummy read to flush:
  SPI.beginTransaction(adsSPI);
  frameNOP();                    // start
  delayMicroseconds(ADSX_TCONV_US);
  (void)frameNOP();              // discard
  SPI.endTransaction();
}

//function returns multiplier to calculate measureed voltage based on ADC input voltage range setting
//first input argument is the ADC's range code and the second argument is for if the range is bipolar
//returns range multiplier as a float
static inline float adsxRangeMultiplier(uint16_t code, bool &isBipolar) {
  switch (code & 0xF) {
    case ADSX_RANGE_BIPOLAR_3X:     isBipolar = true;  return 3.0f;
    case ADSX_RANGE_BIPOLAR_2P5X:   isBipolar = true;  return 2.5f;
    case ADSX_RANGE_BIPOLAR_1P5X:   isBipolar = true;  return 1.5f;
    case ADSX_RANGE_BIPOLAR_1P25X:  isBipolar = true;  return 1.25f;
    case ADSX_RANGE_BIPOLAR_0P625X: isBipolar = true;  return 0.625f;
    case ADSX_RANGE_UNIPOLAR_3X:    isBipolar = false; return 3.0f;
    case ADSX_RANGE_UNIPOLAR_2P5X:  isBipolar = false; return 2.5f;
    case ADSX_RANGE_UNIPOLAR_1P5X:  isBipolar = false; return 1.5f;
    case ADSX_RANGE_UNIPOLAR_1P25X: isBipolar = false; return 1.25f;
    default:                        isBipolar = false; return 1.25f;
  }
}

// Function to convert a raw ADC code to volts for the currently selected range.
// Returns voltage as a float for both unipolar and bipolar ranges.
// LSB = FSR / 2^N; Volts = NFS + code * LSB.
//input argument is measured ADC code
float adsxCodeToVolts(uint32_t codeN) {
  if (codeN > ADSX_CODE_MASK) codeN = ADSX_CODE_MASK;

  bool bipolar;
  const float mult = adsxRangeMultiplier(g_range_code, bipolar);
  const float vref = g_vref_volts;
  const float PFS  = mult * vref;           // +full-scale
  const float NFS  = bipolar ? -PFS : 0.0f; // -full-scale (or 0 for unipolar)
  const float FSR  = PFS - NFS;             // span
  const float LSB  = FSR / (float)(1UL << ADSX_BITS);

  return NFS + (float)codeN * LSB;
}

//function reads ADC and returns ADC value as unsigned 32 int
static uint32_t adsxReadCodeRaw32() {
  SPI.beginTransaction(adsSPI);
  frameNOP();                   // start conversion
  delayMicroseconds(ADSX_TCONV_US);
  uint32_t w = frameNOP();      // returns previous conversion
  SPI.endTransaction();
  return w;
}

//function takes ADC reading and shifts it for proper alignment
//Made for 16 and 18 bit ADC
static uint32_t adsxReadCodeNbits() {
  uint32_t w = adsxReadCodeRaw32();
  return (w >> ADSX_CODE_SHIFT) & ADSX_CODE_MASK;
}

void setup() {
  Serial.begin(115200); //start serial communication
  delay(2000);

  pinMode(PIN_CS, OUTPUT); //setup chip select pin
  digitalWrite(PIN_CS, HIGH);

  if (PIN_RST >= 0) { //setup reset pin if using it
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, HIGH);
    digitalWrite(PIN_RST, LOW);
    delay(1);
    digitalWrite(PIN_RST, HIGH);
  }
  delay(20); // allow POR settle

  SPI.begin(); //start SPI communication with default pins for Arduino board being used

  // Example: use internal 4.096 V ref and set 0..1.25×VREF (≈0..5.12 V)
  adsxSetVrefVolts(4.096f);
  adsxSetRange(g_range_code, /*useInternalRef=*/true);

#if ADSX_ENABLE_TWO_POINT_CAL
  // Compute slope and offset from the two calibration points:
  //   ACTUAL = m * MEASURED + b
  if (ADSX_CAL_P2_MEASURED != ADSX_CAL_P1_MEASURED) {
    g_cal_slope  = (ADSX_CAL_P2_ACTUAL - ADSX_CAL_P1_ACTUAL) /
                   (ADSX_CAL_P2_MEASURED - ADSX_CAL_P1_MEASURED);
    g_cal_offset = ADSX_CAL_P1_ACTUAL - (g_cal_slope * ADSX_CAL_P1_MEASURED);
    g_cal_enabled = true;

    Serial.println(F("Two-point calibration enabled."));
    Serial.print(F("  Calibration range code: 0x"));
    Serial.println(ADSX_CAL_RANGE_CODE, HEX);
    Serial.print(F("  Cal slope  = "));
    Serial.println(g_cal_slope, 8);
    Serial.print(F("  Cal offset = "));
    Serial.println(g_cal_offset, 8);

    if (g_range_code != ADSX_CAL_RANGE_CODE) {
      Serial.println(F("WARNING: g_range_code != ADSX_CAL_RANGE_CODE; "
                       "calibration was collected at a different range."));
    }
  } else {
    g_cal_enabled = false;
    Serial.println(F("Two-point calibration disabled: calibration points are identical."));
  }
#else
  g_cal_enabled = false;
#endif

  Serial.print(F("ADS86"));
  Serial.print(ADSX_BITS);
  Serial.println(F(" configured."));
}

void loop() {
  //This function call gets reading(s) from ADC
  //it will also average multiple readings together based on global reading count setting
  //it will apply the two point calibration gain and offset correction if enabled
  float volts = adsxReadAveragedVolts();  

  Serial.print(F("  Volts: ")); Serial.println(volts, 5);
  // If you also want to see the raw value, uncomment the next two lines:
  // Serial.print(F("  Raw: "));   Serial.println(volts_raw, 6);

  delay(2000); // ~2 s between measurements
}
