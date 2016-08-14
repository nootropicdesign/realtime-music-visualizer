/*
  Realtime music visualizer for LED strips
  Copyright (C) 2016 Michael Krumpus, nootropic design, LLC

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "RealtimeMusicVisualizer.h"
#include <Adafruit_NeoPixel.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <ffft.h>
#include <math.h>

//#define DEBUG
#define ADC_CHANNEL 0
#define N_BANDS 8
#define N_FRAMES 5
#define N_PEAKS 25
#define LED_STRIP_PIN 6

#define N_LEDS 120
#define CUTOFF_BAND 2
#define SINGLE_DIRECTION false
#define MAX_BRIGHTNESS 255
#define MIN_COLOR_CHANGE_COUNT 5
#define MAX_COLOR_CHANGE_COUNT 10

Adafruit_NeoPixel strip = Adafruit_NeoPixel(120, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

peak_t peaks[N_PEAKS];
uint8_t peakIndex = 0;
uint8_t newPeakFlags = 0;
uint8_t randColor[] = {0, 0};
uint8_t randColorCount[] = {0, 0};


// FFT_N = 128
int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N / 2]; // Spectrum output buffer

volatile byte samplePos = 0;     // Buffer position counter
float brightnessScale = MAX_BRIGHTNESS / 255.0;
uint8_t MAX_AGE;

byte
bandPeakLevel[8],      // Peak level of each band
              bandPeakCounter = 0, // Frame counter for delaying the fall of the band peak
              bandPeakDecay = 6, // peak decreases by 1 every bandPeakDecay frames. Larger value is slower decay
              bandCount = 0; // Frame counter for storing past band data
int
band[8][N_FRAMES],   // band levels for the prior N_FRAMES frames
     minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
     maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
     bandDiv[8];    // Used when filtering FFT output to 8 bands

// declare all control variables here so we can more easily measure memory consumption
uint8_t i, j, noiseThreshold, *data, nBins, binNum, weighting, c;
uint16_t minLvl, maxLvl;
int16_t level, sum;

const uint8_t PROGMEM noiseFloor[64] = {8, 6, 6, 5, 3, 4, 4, 4, 3, 4, 4, 3, 2, 3, 3, 4, 2, 1, 2, 1, 3, 2, 3, 2, 1, 2, 3, 1, 2, 3, 4, 4, 3, 2, 2, 2, 2, 2, 2, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4};
const uint8_t PROGMEM eq[64] = {255, 175, 218, 225, 220, 198, 147, 99, 68, 47, 33, 22, 14, 8, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t PROGMEM bandBinCounts[] = {2, 4, 5, 8, 11, 17, 25, 37};
const uint8_t PROGMEM bandBinStarts[] = {1, 1, 2, 3, 5, 7, 11, 16};
const uint8_t PROGMEM band0weights[] = {181, 40};
const uint8_t PROGMEM band1weights[] = {19, 186, 38, 2};
const uint8_t PROGMEM band2weights[] = {11, 176, 118, 16, 1};
const uint8_t PROGMEM band3weights[] = {5, 55, 165, 164, 71, 18, 4, 1};
const uint8_t PROGMEM band4weights[] = {3, 24, 89, 139, 148, 118, 54, 20, 6, 2, 1};
const uint8_t PROGMEM band5weights[] = {2, 9, 29, 70, 125, 172, 185, 162, 118, 74, 41, 21, 10, 5, 2, 1, 1};
const uint8_t PROGMEM band6weights[] = {1, 4, 11, 25, 49, 83, 121, 156, 180, 185, 174, 149, 118, 87, 60, 40, 25, 16, 10, 6, 4, 2, 1, 1, 1};
const uint8_t PROGMEM band7weights[] = {1, 2, 5, 10, 18, 30, 46, 67, 92, 118, 143, 164, 179, 185, 184, 174, 158, 139, 118, 97, 77, 60, 45, 34, 25, 18, 13, 9, 7, 5, 3, 2, 2, 1, 1, 1, 1};
const uint8_t PROGMEM * const bandWeights[] = {band0weights, band1weights, band2weights, band3weights, band4weights, band5weights, band6weights, band7weights};


void setup() {
  randomSeed(analogRead(1));
  memset(bandPeakLevel, 0, sizeof(bandPeakLevel));
  memset(band, 0, sizeof(band));

  for (i = 0; i < N_BANDS; i++) {
    binNum = pgm_read_byte(&bandBinStarts[i]);
    nBins = pgm_read_byte(&bandBinCounts[i]);

    minLvlAvg[i] = 0;
    maxLvlAvg[i] = 512;
    data = (uint8_t *)pgm_read_word(&bandWeights[i]);
    for (bandDiv[i] = 0, j = 0; j < nBins; j++) {
      bandDiv[i] += pgm_read_byte(&data[j]);
    }
  }

  for (i = 0; i < N_PEAKS; i++) {
    peaks[i].age = 0;
    peaks[i].magnitude = 0;
  }
  if (SINGLE_DIRECTION) {
    MAX_AGE = N_LEDS + (N_LEDS / 4);
  } else {
    MAX_AGE = (N_LEDS / 2) + (N_LEDS / 8);
  }

  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin

  sei(); // Enable interrupts

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  TIMSK0 = 0;                // Timer0 off
  startSampling();

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(getMemory());
#endif
}

void loop() {
  // While the ADC interrupt is enabled, wait. The program is still gathering
  // audio samples in the capture buffer.
  while (ADCSRA & _BV(ADIE)) {
    // wait...
  }

  // The sampling interrupt has finished filling the buffer, so show the
  // output we computed last time around the loop. This sends the data to
  // the LED strip.
  strip.show();

  // Perform the FFT algorithm to convert samples to complex numbers.
  fft_input(capture, bfly_buff);

  // Now that we've updated the LED strip and processed the audio samples
  // with FFT, we can resume the collection of audio samples in the sample
  // buffer. The interrupt service routine (ISR) will run while we compute
  // the next LED output based on the audio samples we captured.
  startSampling();

  // Perform the rest of the FFT computation:
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Now call this to analyze the audio. See comments in this function
  // for details.
  analyzeAudioSamples();

  // The peak values for each of the 8 bands has been computed. A bit
  // in the 8-bit value newPeakFlags indicates whether the analysis
  // found a *new* peak in the band.
  for (i = 0; i <= CUTOFF_BAND; i++) {

    // If a new peak was found in band i...
    if (newPeakFlags & (1 << i)) {
      // Map the peak value to a magnitude in range [0,255]. We pass
      // in the band number because the mapping is different for
      // different bands.
      uint8_t magnitude = getMagnitude(i, bandPeakLevel[i]);

      // A nonzero magnitude means that the peak value is large enough to do
      // something visually with it. We ignore small peaks.
      if (magnitude > 0) {
        // We want to store the information about the peak in a
        // peak_t structure.
        // When we actually draw a visualization, the peak_t structures
        // in peaks[] represent the "visually active" band peaks.

        // Look through the list of peak structures 'peaks' for an
        // unused one.
        for (j = 0; j < N_PEAKS; j++) {
          if (peaks[j].magnitude == 0) {
            // unused peak found
            peakIndex = j;
            break;
          }
        }
        // If an unused one not found, we use the last one that was
        // used (peakIndex).

        // Initialize the structure.
        peaks[peakIndex].age = 0;
        // a random component for a visualzation to use. For example,
        // to shift the color a small amount.
        peaks[peakIndex].rnd = random(255); 
        peaks[peakIndex].baseColor = getRandomBaseColor(peaks[peakIndex].rnd);
        peaks[peakIndex].magnitude = magnitude;
      }
    }
  }

  // Clear the last frame.
  strip.clear();

  // Draw the peaks on the LED strip.
  doVisualization();
} // end loop()

void analyzeAudioSamples() {
  // The peaks in each band need to decay so that new peaks can happen later.
  // The value bandPeakDecay determines how fast they decay. Currently set to 6,
  // so every 6th time through the main loop, each band peak decays by 1.
  // If it decays too fast (original value was 3), then there are too many peaks
  // detected and display is messy. Too few and you'll miss some real audio peaks
  // that should be displayed.
  if (++bandPeakCounter >= bandPeakDecay) {
    bandPeakCounter = 0;
    for (i = 0; i <= CUTOFF_BAND; i++) {
      if (bandPeakLevel[i] > 0) {
        bandPeakLevel[i]--;
      }
    }
  }

  // Remove noise and apply EQ levels. This is from the Piccolo project.
  for (i = 0; i < FFT_N / 2; i++) {
    noiseThreshold = pgm_read_byte(&noiseFloor[i]);
    if (spectrum[i] < noiseThreshold) {
      spectrum[i] = 0;
    } else {
      spectrum[i] -= noiseThreshold;
      uint8_t eqVal = pgm_read_byte(&eq[i]);
      if (eqVal > 0) {
        spectrum[i] = (  ( spectrum[i] * (256L - eqVal)) >> 8);
      }
    }
  }

  // Downsample spectrum output to 8 bands.
  newPeakFlags = 0;
  for (i = 0; i <= CUTOFF_BAND; i++) {
    data   = (uint8_t *)pgm_read_word(&bandWeights[i]);
    binNum = pgm_read_byte(&bandBinStarts[i]);
    nBins = pgm_read_byte(&bandBinCounts[i]);
    for (sum = 0, j = 0; j < nBins; j++) {
      sum += spectrum[binNum++] * pgm_read_byte(&data[j]);
    }
    band[i][bandCount] = sum / bandDiv[i];
    minLvl = maxLvl = band[i][0];
    for (j = 1; j < N_FRAMES; j++) { // Get range of prior frames
      if (band[i][j] < minLvl)      minLvl = band[i][j];
      else if (band[i][j] > maxLvl) maxLvl = band[i][j];
    }
    if ((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    minLvlAvg[i] = (minLvlAvg[i] * 7 + minLvl) >> 3; // Dampen min/max levels
    maxLvlAvg[i] = (maxLvlAvg[i] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 5L * (band[i][bandCount] - minLvlAvg[i]) /
            (long)(maxLvlAvg[i] - minLvlAvg[i]);

    if (level < 0L)      c = 0;
    else if (level > 20) c = 20;
    else                 c = (uint8_t)level;

    if (c > bandPeakLevel[i]) {
      bandPeakLevel[i] = c; // New peak found for this frequency band.
      newPeakFlags = newPeakFlags | (1 << i); // set new peak flag for this band
    }
  }

  if (++bandCount >= N_FRAMES) bandCount = 0;

}

void doVisualization() {
  float ageScale, b;
  uint32_t color;
  uint8_t r;

  for (i = 0; i < N_PEAKS; i++) {
    if (peaks[i].magnitude > 0) {
      // peak is visually active

      // The age of the visual peak will be used to determine brightness.
      ageScale = (float)(1.0 - ((float)peaks[i].age / (float)MAX_AGE));

      // If brightness has been altered by a parameter, scale everything further.
      // Get the right color for this peak's band and adjust the brightness.
      color = adjustBrightness(getColor(peaks[i].baseColor, peaks[i].rnd), ageScale * brightnessScale);

      // Calculate horizontal position relative to starting point.
      // The first term is a value from 0.0 to 1.0 indicating speed.
      // It is a function of the peak's magnitude.
      // Minimum speed is 0.5 (127.0 / 255.0). Maximum is 1.0 (255.0 / 255.0);
      // So speed is how many LEDs does it "move" every step. Multiply this by
      // the peak's age to get the position.
      uint8_t pos;
      if (peaks[i].magnitude > 150) {
        pos = peaks[i].age;
      } else {
        pos = ((127 + (peaks[i].magnitude / 2)) / 255.0) * peaks[i].age;
      }

      if (!SINGLE_DIRECTION) {
        // Draw one right of center and one left of center.
        strip.setPixelColor((N_LEDS / 2) + pos, color);
        strip.setPixelColor(((N_LEDS / 2) - 1) - pos, color);
      } else {
        // Draw pixel relative to 0
        strip.setPixelColor(pos, color);
      }

      // Increment age
      peaks[i].age++;

      // If too old, retire this peak, making it available again.
      if (peaks[i].age > MAX_AGE) {
        peaks[i].magnitude = 0;
        continue;
      }

      if ((pos >= ((N_LEDS / 2) - 1)) && (!SINGLE_DIRECTION)) {
        // Off the edge of the strip. This peak can now be returned to the pool
        // of peak structures that are available for use.
        peaks[i].magnitude = 0;
      }
      if ((pos >= (N_LEDS-1)) && (SINGLE_DIRECTION)) {
        // Off the edge of the strip. This peak can now be returned to the pool
        // of peak structures that are available for use.
        peaks[i].magnitude = 0;
      }
    }
  }
}

// Get a random color. There are 2 random colors that change over
// time depending on MIN_COLOR_CHANGE_COUNT and MAX_COLOR_CHANGE_COUNT.
// This gives a 2-color scheme that changes over time.
uint8_t getRandomBaseColor(uint8_t r) {
  if ((r % 2) == 0) {
    if (randColorCount[0]-- == 0) {
      randColorCount[0] = random(MIN_COLOR_CHANGE_COUNT, MAX_COLOR_CHANGE_COUNT+1);
      randColor[0] = random(255);
    }
    return randColor[0];
  } else {
    if (randColorCount[1]-- == 0) {
      randColorCount[1] = random(MIN_COLOR_CHANGE_COUNT, MAX_COLOR_CHANGE_COUNT+1);
      randColor[1] = random(255);
    }
    return randColor[1];
  }

}

// Map a color index in range [0-255] to a color.
// This is a uniform colorspace over 8 base colors:
// [red, orange, yellow, green, cyan, blue, magenta, white]
uint32_t getColor(uint8_t index, uint8_t rnd) {
  byte r, g, b;

  index += (rnd >> 5); // introduce random variation in the color.
  byte f = (index % 32) * 8; // fractional part of color component
  switch (index / 32) {
    case 0: // red
      r = 255;
      g = f >> 1;
      b = 0;
      break;
    case 1: // orange
      r = 255;
      g = 127 + (f >> 1);
      b = 0;
      break;
    case 2: // yellow
      r = 255 - f;
      g = 255;
      b = 0;
      break;
    case 3: // green
      r = 0;
      g = 255;
      b = f;
      break;
    case 4: // cyan
      r = 0;
      g = 255 - f;
      b = 255;
      break;
    case 5: // blue
      r = f;
      g = 0;
      b = 255;
      break;
    case 6: // magenta
      r = 255 - f / 2;
      g = f / 2;
      b = 255 - f / 2;
      break;
    case 7: // white
      r = 127 + f / 2;
      g = 127 - f / 2;
      b = 127 - f / 2;
      break;
  }

  if (MAX_BRIGHTNESS < 255) {
    // scale by brightnessScale;
    // optimize for 255 and 0 cases to avoid floating point multiply
    if (r == 255) {
      r = MAX_BRIGHTNESS;
    } else {
      if (r > 0) {
        r = ((float)r * brightnessScale);
      }
    }
    if (g == 255) {
      g = MAX_BRIGHTNESS;
    } else {
      if (g > 0) {
        g = ((float)g * brightnessScale);
      }
    }
    if (b == 255) {
      b = MAX_BRIGHTNESS;
    } else {
      if (b > 0) {
        b = ((float)b * brightnessScale);
      }
    }
  }
  return strip.Color(r, g, b);
}


// Map the band peak level to a magnitude in range [0,255].
// This mapping depends on band number because different bands have different
// characteristics.
uint8_t getMagnitude(uint8_t band, uint8_t peakValue) {
  uint8_t peakMin, peakMax;
  switch (band) {
    case 0:
      // bass bands are really strong, so don't show anything with peak value
      // smaller than 8. They can have high values, so anything bigger than 20 is
      // maximum magnitude.
      peakMin = 8;
      peakMax = 20;
      break;
    case 1:
      peakMin = 8;
      peakMax = 20;
      break;
    case 2: // fall through...
    case 3:
    case 4:
    case 5:
      // These middle bands need a little more sensitivity, so allow values as low
      // as 4 to be represented visually.
      // These bands never go that high, so let's say that value of 8 is the high end.
      peakMin = 4;
      peakMax = 8;
      break;
    case 6:
    case 7:
      // High bands are not so sensitive and go a bit higher in practice.
      peakMin = 6;
      peakMax = 12;
      break;
  }

  // If below min, then return 0 so we don't do anything visually.
  if (peakValue < peakMin) {
    return 0;
  }
  // Constrain to range [peakMin,peakMax]
  peakValue = constrain(peakValue, peakMin, peakMax);

  // Map to range [1,255] for all bands.
  return map(peakValue, peakMin, peakMax, 1, 255);
}


// Configure the ADC to be free-running. It samples constantly and calls the ISR when
// a new conversion is ready to be recorded.
void startSampling() {
  samplePos = 0; // Reset sample counter
  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  // Start a conversion. We want to discard the first conversion after
  // changing the voltage reference reference.
  ADCSRB = 0;
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC);   // ADC start

  // Wait until conversion finishes.
  while (ADCSRA & _BV(ADSC));

  // Now start auto-triggered conversions with interrupt enabled.
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
}


// Audio-sampling interrupt. This is invoked automatically whenever an ADC
// conversion is ready.
ISR(ADC_vect) {
  static const int16_t noiseThreshold = 4; // ignore small voltage variations.

  // the sample is available in the ADC register
  int16_t sample = ADC; // 0-1023

  capture[samplePos] =
    ((sample > (512 - noiseThreshold)) &&
     (sample < (512 + noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if (++samplePos >= FFT_N) {
    // The sample buffer is full, so disable the interrupt
    ADCSRA &= ~_BV(ADIE);
  }
}

// Scale brightness of a color.
uint32_t adjustBrightness(uint32_t c, float amt) {

  // pull the R,G,B components out of the 32bit color value
  uint8_t r = (uint8_t)(c >> 16);
  uint8_t g = (uint8_t)(c >> 8);
  uint8_t b = (uint8_t)c;

  // Scale
  r = r * amt;
  g = g * amt;
  b = b * amt;

  // Pack them into a 32bit color value again.
  return strip.Color(r, g, b);
}

