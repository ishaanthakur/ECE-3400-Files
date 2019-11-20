/* Test tone detection using the Open Music Labs FFT library 
 * The FFT algorithm is a method for computing the DFT of a signal in n log n time. It is a divide and conquer algorithm that recursively calculates
 * the DFT of interleaving portions of the signal, and merges the results to obtain the final output. It is more efficient than the definition (n^2),
 * for cases when the entire DFT, and not just one bin has to be computed.
*/


#define LIN_OUT 1 // use the linear output function
#define FFT_N 256 // set to 256 point fft

#include <FFT.h>

void setup() {
  Serial.begin(115200); // set serial baud rate to 115200 

  ADMUX = 1 << REFS0; // use Vcc as analog reference
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS1); // enable, start, and set the adc to free running mode. set prescaler to 64
  ADCSRB = 0;
  DIDR0 = (1 << ADC0D); // disable digital functionalty on A0
}

void loop() {
  while(1) {
    cli(); // disable interrupts for speed
    
    for (uint16_t sample_num = 0 ; sample_num < FFT_N; sample_num++) {
      while(!(ADCSRA & (1 << ADIF)));
      ADCSRA |= 1 << ADIF;

      int16_t sample = ADCL; // ADCL must be read first
      sample |= ADCH << 8;
      sample -= 512; // remove dc bias
      sample <<= 6; // map [-512, 511] -> [-32768, 32767] for better accuracy in fixed point calculations 
      
      fft_input[sample_num * 2] = sample; // even bins should be filled with time domain data.
      fft_input[sample_num * 2 + 1] = 0; // odd bins are reserved for the imaginary part of the complex output
    }
    
    fft_window(); // window the data with non-rectangular window for better results
    fft_reorder(); // reorder the data before doing the fft
    fft_run(); // process the data in the fft
    fft_mag_lin(); // take the output of the fft
    
    sei();

    // transmit all of the dft bins
    for(uint16_t bin_num = 0; bin_num < FFT_N / 2; bin_num++) {
      // transmit in big endian
      Serial.write((uint8_t)(fft_lin_out[bin_num]));
      Serial.write((uint8_t)(fft_lin_out[bin_num] >> 8));
    }
    
    delay(100); // delay one decisecond
  }
}
