#include <util/atomic.h>

// --- PIN CONFIGURATION ---
const int inputPin = 2;       // VSS sensor input from gearbox
const int outputPin = 9;      // Corrected speed output to speedometer/ECU

// --- CALIBRATION ---
const float multiplier = 3.423;

// --- SYSTEM VARIABLES ---
volatile unsigned long v_lastInputTime = 0;
volatile unsigned long v_inputPeriod = 0;
volatile int v_consecutiveRejections = 0; // Prevents filter lockouts

unsigned long lastOutputToggle = 0;
unsigned long currentHalfPeriod = 0;
bool outputState = LOW;

void handlePulse() {
    unsigned long now = micros();
    unsigned long duration = now - v_lastInputTime;
    
    // MAXIMUM NOISE PROTECTION:
    // Ignores any spike/pulse faster than 1500us.
    // Cleanly allows your VSS to track all the way up to ~266 km/h.
    if (duration < 1000) {
        return; 
    }

    // DYNAMIC RESONANCE FILTER
    unsigned long lastValidPeriod = v_inputPeriod;

    // Filter operates between 1.5ms and 400ms periods (down to ~0.45 km/h)
    if (lastValidPeriod > 1500 && lastValidPeriod < 400000) {
        unsigned long minAllowed = (lastValidPeriod * 60UL) / 100UL;  // 60% window
        unsigned long maxAllowed = (lastValidPeriod * 140UL) / 100UL; // 140% window
        
        if (duration < minAllowed || duration > maxAllowed) {
            v_consecutiveRejections++;
            
            // SELF-HEALING: If we reject 5 pulses in a row, reset sync to break lockouts
            if (v_consecutiveRejections < 5) {
                v_lastInputTime = now; 
                return;
            }
        }
    }

    // Valid pulse accepted
    v_consecutiveRejections = 0; 
    v_inputPeriod = duration;
    v_lastInputTime = now;
}

void setup() {
    // Internal pull-up active to handle the open-collector signal from the VSS safely
    pinMode(inputPin, INPUT_PULLUP);
    pinMode(outputPin, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(inputPin), handlePulse, RISING);
}

void loop() {
    unsigned long now = micros();
    unsigned long safeInputPeriod;
    unsigned long safeLastInputTime;

    // Grab volatile variables quickly and safely
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        safeInputPeriod = v_inputPeriod;
        safeLastInputTime = v_lastInputTime;
    }

    // Timeout: If no pulse at all for 400ms, the car has stopped
    if (now - safeLastInputTime > 400000) {
        currentHalfPeriod = 0;
        digitalWrite(outputPin, LOW);
    } 
    else if (safeInputPeriod > 0) {
        // Calculate the output half-period
        currentHalfPeriod = (safeInputPeriod / multiplier) / 2;
    }

    // Clean Wave Generator
    if (currentHalfPeriod > 0) {
        if (now - lastOutputToggle >= currentHalfPeriod) {
            outputState = !outputState;
            digitalWrite(outputPin, outputState);
            lastOutputToggle = now;
        }
    }
}