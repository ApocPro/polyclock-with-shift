/*
Alternate firmware for the ginky synthese grains eurorack module
Code by a773 (atte.dk) and released under the GPL licence
*/

/* 11-9-2021 Adapted by Jesse Stevens of artist duo Cake Industries for Look Mum No Computer offbeat shift needs */
/* 16-10-2021 Further changes to allow for longer gaps between incoming beats and logic to handle multi/div changes between beats for Look Mum No Computer */
/* 24-10-2021 Reworked by Rohan Mitchell for easier multi/div changes between beats */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


#define TRIGGER_LENGTH  20
#define MULT_POT        2
// adding a switch to increase the multiplication factor
#define MULT_SWITCH     4
#define DIV_POT         1
// adding a switch to add Sloth and Glacial Modes to division factor
#define DIV_SWITCH      6   
#define SHORT           0
#define SLOTH           1
#define GLACIAL         2     
#define MODE_POT        0
#define BEATSHIFT_POT   5
#define CLOCK_IN        3
#define UPPER_POT_MAX   1024
#define MIDDLE_POT_MAX  1024
#define LOWER_POT_MAX   1024
#define NB_POT_SLICES   10
// adding switch slices
#define NB_SWITCH_SLICES 3
#define SWITCH_MAX      1024
/*
#define MODE_SIMPLE     0
#define MODE_COMPLEX    1
*/
//changing modes
#define MODE_DIV_MULT   0
#define MODE_POLY       1
#define MODE_EUCLID     2
#define SHIFTED_OUT     11
#define UNSHIFTED_OUT   10
#define KNOB_READING_INTERVAL 250


/**
 * Interface for reading and interpreting the control knobs.
 */
class Controls {
private:
  // new factors for DIV/MULT mode
  const int SHORT_FACTORS[10] = {1,2,3,4,5,6,7,8,9,10};
  const int SLOTH_FACTORS[10] = {1,2,3,4,6,8,12,16,24,32};
  const int GLACIAL_FACTORS[10] = {1,2,4,8,16,32,64,128,256,512};
  const int DIV_FACTORS_ARRAY[3][10] = {{SHORT_FACTORS}, {SLOTH_FACTORS}, {GLACIAL_FACTORS}};
  
  // adding the DIV/MULT switch Factors
  const int DIV_MODE[3] = {SHORT, SLOTH, GLACIAL};
  const int MULT_MODE[3] = {1,2,3};

  // new factors for the POLY mode 
  const int NUMERATOR[10] = {2,3,4,5,6,7,9,11,13,15};
  const int DENOMINATOR[3] = {4,8,16};

  /* original dividing factors
  const int SIMPLE_FACTORS[10] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512}; 
  const int COMPLEX_FACTORS[10] = {1, 3, 5, 7, 11, 13, 17, 19, 23, 29};
  */

public:
  int mult_reading;
  int div_reading;
  int mode_reading;
  float beatshift;
  // adding the variables for the switches
  int div_mode_reading;
  int mult_mode_reading;

  int mode;
  bool stopped;


  Controls() {
    mode = -1;
    stopped = false;
  }

  void setup() {
    pinMode(CLOCK_IN, INPUT_PULLUP);
    pinMode(SHIFTED_OUT, OUTPUT);
    pinMode(UNSHIFTED_OUT, OUTPUT);
  }

  void read() {
    mult_reading = analogRead(MULT_POT);
    div_reading = analogRead(DIV_POT);
    mode_reading = analogRead(MODE_POT);
    beatshift = analogRead(BEATSHIFT_POT);
    // adding the read values for the switches
    div_mode_reading = analogRead(DIV_SWITCH);
    mult_mode_reading = analogRead(MULT_SWITCH);

    Serial.print(get_mult());
    Serial.print(", ");
    Serial.print(get_div());
    Serial.print(", ");
    Serial.println(get_beatshift());
  }

  void updateSettings(bool edge) {
    if (mode_reading < LOWER_POT_MAX/3) {
      // CCW DIV_MULT_MODE
      if(edge) {
        mode = MODE_DIV_MULT;
        stopped = false;
      }
      /*
      // CCW simple mode
      if(edge) {
        mode = MODE_SIMPLE;
        stopped = false;
      }
      */
    } 
    else if(mode_reading > LOWER_POT_MAX/3*2) {
      if(edge) {
        mode = MODE_POLY;
        stopped = false;
      }
      /*
      // CW complex mode
      if(edge) {
        mode = MODE_COMPLEX;
        stopped = false;
      }
      */
    } 
    else {
      // stopped
      stopped = true;
    }
  }

  /**
   * Fetch the multiplication factor
   * Output clock frequency = input clock frequency * multiplication factor
   * combined with the switch multiplier.
   * This value is controlled by the upper pot. We divide the pot's range
   * into NB_POT_SLICES slices, and then use those to look up a factor from
   * either simple_factors (powers of two) or complex_factors (prime numbers).
   */
  int get_mult() {
    bool div = false;
    int slice = mult_reading * (NB_POT_SLICES) / UPPER_POT_MAX;
    // including switch slice
    int switch_slice = mult_mode_reading * (NB_SWITCH_SLICES) / SWITCH_MAX;  // we need to remember a pulldown resistor on this pin
    return slice2factor(slice, switch_slice, mode, div);
  }

  /**
   * Fetch the division factor
   * Output clock frequency = input clock frequency / division factor
   * combined with switch which determines which range the divison factor should use.
   * This value is controlled by the middle pot. We divide the pot's range
   * into NB_POT_SLICES slices, and then use those to look up a factor from
   * either simple_factors (powers of two) or complex_factors (prime numbers).
   */
  int get_div() {
    bool div = true;
    int slice = div_reading * (NB_POT_SLICES) / MIDDLE_POT_MAX;
    // including switch slice
    int switch_slice = div_mode_reading * (NB_SWITCH_SLICES) / SWITCH_MAX;  // we need to remember a pulldown resistor on this pin
    int factor = slice2factor(slice, switch_slice, mode, div);

    return factor;
  }

  /**
   * Fetch the beatshift factor (scaled 0-1)
   * This value is controlled by the lower pot. Of the 0-1023 range, we introduce
   * a dead zone from 0-30, and then linearly interpolate from 0-1 for the
   * remainder of the range.
   */
  float get_beatshift() {
    if (beatshift < 30) {
      return 0;
    } else {
      return map(beatshift, 30, 1023, 0, 95) / 100.0;
    }
  }

private:
  int slice2factor(int slice, int switch_slice, int mode, bool div) {
    if(mode == MODE_DIV_MULT) {
      if(div) {
        return DIV_FACTORS_ARRAY[switch_slice][slice];
      }
      else {
        return (SHORT_FACTORS[slice]*MULT_MODE[switch_slice]);
      }
    }
    /*
     The math of the Poly Mode is a little convoluted.  We are looking at
     2 time signatures.  The time signature from Division pot and switch represent 
     the input time signature. (I.e NUMERATOR[2]=4 and DENOMINATOR[0]=4 would be 4/4)
     The time signature for the output time is the same but from the Mult pot. (I.e. 3/4)
     So if we want to find the down beat of the input clock in 4/4 time we need to divide 
     the input frequency by 4.  Now we determine the frequency of the output in 3/4 time 
     by simply multiplying by 3.  But what if we want 4/4:3/8?  Basically this is double
     4/4:3/4.  We solve this value by multiplying both frequencies by the Denominator as well.
     Say 120 bpm in 4/4 and we want a frequency that would be 3/8 with the same downbeat.
     120/(4*4)=7.5*(3*8)= 180 bpm.  Basically the denominator determines a .25x, .5x, 2x, or 4x 
     factor of the final result, but we split the function in 2 parts that make it appear 
     a little strange in the programming.
     */
    elif(mode == MODE_POLY) {
      return (NUMERATOR[slice]*DENOMINATOR[switch_slice]);
    }
    // Euclidean Mode, still needs to be written
    else {
      return;
    }
    /*
    if(mode == MODE_SIMPLE) {
      return SIMPLE_FACTORS[slice];
    } else {
      return COMPLEX_FACTORS[slice];
    }*/
  }
};


/**
 * GateReader reads the clock pin and detects edges.
 */
class GateReader {
private:
  bool clock_high;

public:
  GateReader() {
    clock_high = false;
  }

  bool readEdge(long now) {
    int gate = digitalRead(CLOCK_IN);

    bool edge = false;

    // My setup is reverse logic trigger (using NPN transistor as buffer on input)
    if (gate == LOW && !clock_high) {
      edge = true;
    }
    clock_high = gate == LOW;

    return edge;
  }
};


/**
 * Given control inputs and fed input edges, TimeKeeper will report the output
 * wavelength and when to fire an output trigger (taking mult and div factors
 * into account).
 */
class TimeKeeper {
private:
  Controls* controls;
  int edge_count;
  long last_edge;
  int last_div;
  long last_phrase_start;
  long last_scaled_time;
  long wavelength;

  bool was_in_output_pulse;
  bool fire_trigger;
  long last_trigger;

public:
  TimeKeeper(Controls* controls) {
    this->controls = controls;
    edge_count = 0;
    last_edge = 0;
    last_div = 0;
    last_phrase_start = 0;
    wavelength = 0;

    was_in_output_pulse = false;
    fire_trigger = false;
    last_trigger = 0;
  }

  void update(long now, bool edge) {
    if (edge) {
      processEdge(now);
    }

    bool outputEdge = this->outputEdge(now);

    // Detect start of phrase
    if (edge && now > readyForEndOfPhrase()) {
      last_phrase_start = now;
      last_scaled_time = 0;
    }

    handleDivReduction(now);

    fire_trigger = false;
    if (debounce(now, haveWavelength() && outputEdge)) {
      fire_trigger = true;
      last_trigger = now;
    }
  }

  long outputWavelength() {
    return float(wavelength) / controls->get_mult();
  }

  // Returns whether to fire the output trigger
  bool fireTrigger() {
    return fire_trigger;
  }

private:
  // Accept an input clock and keep track of wavelength
  void processEdge(long now) {
    if (last_edge != 0) {
      wavelength = now - last_edge;
    }

    last_edge = now;
  }

  /**
   * To determine when to show output pulses, we sample an imaginary output waveform
   * (a square wave at 50% duty cycle of the desired frequency), and then perform
   * edge detection. This method will return true when we detect a leading edge of this
   * imaginary output waveform.
   */
  bool outputEdge(long now) {
    bool edge = false;

    if (inOutputPulse(now)) {
      if (!was_in_output_pulse) {
        was_in_output_pulse = true;
        edge = true;
      }
    } else {
      was_in_output_pulse = false;
    }

    return edge;
  }

  // Skip double-triggers
  bool debounce(long now, bool trigger) {
    if (trigger) {
      int trigger_period = now - last_trigger;
      if (trigger_period < beatLength() * 0.8) {
        return false;
      }
    }

    return trigger;
  }

  /**
   * Sample an imaginary output waveform - a square wave at 50% duty cycle, with a wavelength
   * equal to input_wavelength * multiplication_factor.
   */
  bool inOutputPulse(long now) {
    long offset = now - last_phrase_start;
    float relative_time = offset / float(wavelength);
    long scaled_time = relative_time * scaleFactor() * 2;

    // Given a wavelength of 100ms and a multiplication factor of 2,
    // our relative fraction and modulo values will be:
    //   0 ms: 0; 0 % 2 = 0
    //  25 ms: 1; 1 % 2 = 1
    //  50 ms: 2; 2 % 2 = 0
    //  75 ms: 3; 3 % 2 = 1
    // 100 ms: 4; 4 % 2 = 0
    // In this example, this gives us a new wavelength of 50ms

    if (pastEndOfPhrase(now)) {
      return false;
    } else {
      return scaled_time % 2 == 0;
    }
  }

  // If divider has just been reduced, our phrase length has also
  // been reduced, which may well leave us past the end of phrase.
  // If this happens we would stop outputting triggers until the start
  // of the next phrase, which would cause an undesired silence.
  // Prevent this by resetting the start of the phrase.
  void handleDivReduction(long now) {
    if (divReducedThisFrame() && pastEndOfPhrase(now)) {
      last_phrase_start = last_edge;
    }
  }

  bool divReducedThisFrame() {
    bool result = last_div > controls->get_div();
    last_div = controls->get_div();

    return result;
  }

  bool pastEndOfPhrase(long now) {
    return now >= last_phrase_start + phraseLength();
  }

  /**
   * Return a time when we've received all our clocks for the phrase
   * and the next clock we get should be considered the start of the
   * next phrase.
   */
  long readyForEndOfPhrase() {
    long delta = wavelength / 2;
    return last_phrase_start + phraseLength() - delta;
  }

  // Returns whether we've determined a wavelength reading
  bool haveWavelength() {
    return wavelength > 0;
  }

  float scaleFactor() {
    return controls->get_mult() / float(controls->get_div());
  }

  long beatLength() {
    return wavelength / scaleFactor();
  }

  long phraseLength() {
    return long(wavelength) * controls->get_div();
  }
};


class TimeFollower {
private:
  Controls* controls;
  long last_signal;
  long wavelength;
  bool output_fired;

public:
  TimeFollower(Controls* controls) {
    this->controls = controls;
    last_signal = 0;
    wavelength = 0;
    output_fired = false;
  }

  bool shouldFire(long now, bool signal) {
    // Detect edges and track last signal
    if (signal) {
      if (last_signal > 0) {
        wavelength = now - last_signal;
      }
      last_signal = now;
      output_fired = false;
    }

    // Fire output
    if (now >= last_signal + delayTime() && !output_fired) {
      output_fired = true;
      return true;
    }

    return false;
  }

private:
  long delayTime() {
    return wavelength * controls->get_beatshift();
  }
};


class Trigger {
private:
  int pin;
  int triggerlength;
  bool clock_high;

public:
  long last_trigger_out;

  Trigger(int pin) {
    this->pin = pin;
    triggerlength = TRIGGER_LENGTH;
    clock_high = false;
    last_trigger_out = 0;
  }

  // Fire the trigger, for length in ms
  void fire(long now, int triggerlength) {
    digitalWrite(pin, HIGH);
    this->triggerlength = triggerlength;
    clock_high = true;
    last_trigger_out = now;
  }

  // Update the trigger, setting pin to LOW when duration has expired
  void update(long now) {
    if( ((now - last_trigger_out) > triggerlength) && clock_high) {
      digitalWrite(pin, LOW);
      clock_high = false;
    }
  }
};



long now = 0;
unsigned long last_knob_read = 0;


GateReader gateReader;
Controls controls;
TimeKeeper timeKeeper(&controls);
TimeFollower timeFollower(&controls);
Trigger unshiftedTrigger(UNSHIFTED_OUT);
Trigger shiftedTrigger(SHIFTED_OUT);


void setup() {
  controls.setup();
  controls.read();
  Serial.begin(115200);
}

void loop()
{
  // Store the current time:
  now = millis();

  // Read the gate edge into "edge" variable (find edge of incoming gate):
  bool edge = gateReader.readEdge(now);

  // Check if we're in simple or complex numbers mode:
  controls.updateSettings(edge);

  // If it's been long enough, update readings from knobs for multi/div:
  if (now - last_knob_read >= KNOB_READING_INTERVAL) {
    controls.read();
    last_knob_read = now;
  }

  // Vary the output trigger length to make sure when we're going fast we don't just keep outputs on:
  int trigger_length = min(TRIGGER_LENGTH, timeKeeper.outputWavelength() / 2);

  // Trigger update
  unshiftedTrigger.update(now);
  shiftedTrigger.update(now);

  // Fire unshifted trigger
  timeKeeper.update(now, edge);
  if (timeKeeper.fireTrigger()) {
    unshiftedTrigger.fire(now, trigger_length);
  }

  // Fire shifted trigger
  if (timeFollower.shouldFire(now, timeKeeper.fireTrigger())) {
    shiftedTrigger.fire(now, trigger_length);
  }


}
