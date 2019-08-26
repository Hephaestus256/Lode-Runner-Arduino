const int ledPin = 13;
int duration_count;
bool stat = false;
int tune_data[128 * 2];
int note_n = 0;
int note_ct = 0;
int note_delta = 0;
int note_rep = -1;

inline int note_to_freq(int note, int octave = 0, int step = 0)
{
  return int(440.0 * pow(1.05946309, note + 12 * octave) * pow(1.00724641223704, step));
}

inline int beat_to_dur(int quarter, bool and_a_half = false, int q = 200)
{
  return quarter * q + (int(and_a_half) * q / 2);
}

// song: charge
// complete level 1
void song_charge()
{
  init_note(note_to_freq(-3), beat_to_dur(1));
  init_note(note_to_freq(3), beat_to_dur(1));
  init_note(note_to_freq(7), beat_to_dur(1));
  init_note(note_to_freq(11), beat_to_dur(2));
  init_note(note_to_freq(7), beat_to_dur(1));
  init_note(note_to_freq(11), beat_to_dur(2, true));
}

// song: shave and a haircut
// complete level 3
void song_shave()
{
  int q = 100;
  
  init_note(note_to_freq(-2), beat_to_dur(2, false, q));
  init_note(0, 25);
  init_note(note_to_freq(-5), beat_to_dur(1, false, q));
  init_note(0, 25);
  init_note(note_to_freq(-5), beat_to_dur(1, false, q));
  init_note(0, 25);
  init_note(note_to_freq(-4), beat_to_dur(2, false, q));
  init_note(0, 25);
  init_note(note_to_freq(-5), beat_to_dur(2, false, q));
  init_note(0, beat_to_dur(2));
  init_note(note_to_freq(-3), beat_to_dur(2, false, q));
  init_note(0, 25);
  init_note(note_to_freq(-2), beat_to_dur(2, false, q));
}

// song: first call
void song_first()
{
  
}

// i have no idea what this song is called
// complete level 2 song
void song_gallup()
{
  const int q = 75;
  
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3, 0, 1), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 150);
  
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3, 0, 1), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 150);
  
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3, 0, 1), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3), beat_to_dur(2, false, q));
  init_note(0, 50);
  
  init_note(note_to_freq(-1), beat_to_dur(2, false, q));
  init_note(0, 50);
  init_note(note_to_freq(1), beat_to_dur(2, false, q));
  init_note(0, 50);

  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3, 0, 1), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 150);
  
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3, 0, 1), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 150);
  
  init_note(note_to_freq(-3), beat_to_dur(1, false, q));
  init_note(0, 15);
  init_note(note_to_freq(5), beat_to_dur(1, false, q));
  init_note(0, 15);

  init_note(note_to_freq(3), beat_to_dur(2, false, q));
  init_note(0, 50);
  init_note(note_to_freq(0), beat_to_dur(2, false, q));
  init_note(0, 50);
  init_note(note_to_freq(-3), beat_to_dur(2, false, q));
  init_note(0, 50);

}

void sound_fall()
{
  const int s = 40;
  
  for (int i = 0; i < 16; i++) {
    init_note(note_to_freq(12 - i, 0, 7), s);
    init_note(note_to_freq(12 - i, 0, 6), s);
    init_note(note_to_freq(12 - i, 0, 5), s);
    init_note(note_to_freq(12 - i, 0, 4), s);
    init_note(note_to_freq(12 - i, 0, 3), s);
    init_note(note_to_freq(12 - i, 0, 2), s);
    init_note(note_to_freq(12 - i, 0, 1), s);
    init_note(note_to_freq(12 - i, 0, 0), s);
  }
}

void sound_die()
{
  const int s = 12;
  
  for (int i = 0; i < 32; i++) {
    init_note(note_to_freq(20 - i * 1, 0, 0), s);
  }
}

void sound_goal()
{
  init_note(note_to_freq(2, 0, 0), 100);
  init_note(note_to_freq(6, 0, 0), 100);
  init_note(note_to_freq(10, 0, 0), 100);
}

void sound_dig()
{
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      init_note(note_to_freq(12 - j * 4, 0, 0), 25);
    }
    init_note(0, 25);
  }
}

void wait_tune_done()
{
  while(tune_playing());
  delay(500);
}

void sound_finish()
{
  for (int i = 0; i < 24; i++) {
    init_note(note_to_freq(i / 2, 0, 0), 50);
    init_note(0, 30);
  }
}

void sound_test()
{
  init_note(note_to_freq(-3), beat_to_dur(1, false));
  init_note(0, 15);
  init_note(note_to_freq(-1), beat_to_dur(1, false));
  init_note(0, 15);
  init_note(note_to_freq(1), beat_to_dur(1, false));
  init_note(0, 15);
  
  // void rep_note(int ct, int to_index, int delta)
  rep_note(3, 0, 0);

  init_note(note_to_freq(-1), beat_to_dur(1, false));
  init_note(note_to_freq(1), beat_to_dur(1, false));
  init_note(note_to_freq(3), beat_to_dur(1, false));
}

void setup()
{
  Serial.begin(9600);
  Serial.println("begin");

  init_tune();
  sound_test();
  start_tune();

  wait_tune_done();
  
  init_tune();
  sound_finish();
  start_tune();
  
  wait_tune_done();

  //*
  init_tune();
  song_gallup();
  start_tune();
  
  wait_tune_done();

  init_tune();
  song_shave();
  start_tune();
  
  wait_tune_done();

  init_tune();
  sound_dig();
  start_tune();

  wait_tune_done();

  init_tune();
  sound_goal();
  start_tune();

  wait_tune_done();

  init_tune();
  song_charge();
  start_tune();

  wait_tune_done();

  init_tune();
  sound_fall();
  start_tune();

  wait_tune_done();
  //*/
  
  init_tune();
  sound_die();
  start_tune();

  wait_tune_done();

}

void start_tune()
{
  cli(); // stop interrupts
  stop_tune();
  note_n = 0;
  duration_count = 0;
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  next_note();
  sei(); // start interrupts
}

void stop_tune()
{
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // set prescaler 1 to zero to stop
}

void init_note(int freq, int dur)
{
  tune_data[note_ct] = freq;
  tune_data[note_ct + 1] = dur;
  note_ct += 2;
}

void rep_note(int ct, int to_index, int delta)
{
  tune_data[note_ct] = -1 - to_index;
  tune_data[note_ct + 1] = ct;
  note_ct += 2;
}

void init_tune()
{
  pinMode(ledPin, OUTPUT);
  stop_tune();
  note_n = -1;
  note_ct = 0;
  duration_count = 0;
  stat = false;
}

void play_note(int freq, int dur)
{
  //Serial.println(freq);
  
  TCNT1 = 0; // initialize counter value to 0

  if (freq == 0) { // silent note
    duration_count = dur;
    OCR1A = 16000 - 1; // set compare match register for X Hz
    TCCR1B = (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
  } else { // audible note
    long int ps_n;
    int ps_bits;
    
    ps_bits = 1 << WGM12;
  
    if (freq >= 245) {
      // prescaler of 1
      ps_n = 1;
      ps_bits |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    } else if (freq >= 31) {
      // prescaler of 8
      ps_n = 8;
      ps_bits |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    }
    
    duration_count = int( 2L * (long(dur) * long(freq) * long(1)) / 1000L );
    OCR1A = 16000000L / (ps_n * 2 * freq) - 1; // set compare match register for X Hz
    TCCR1B = ps_bits;
  }
}

void pulse_spkr()
{
  if (tune_data[note_n - 2]) {
    digitalWrite(ledPin, stat);
    stat = !stat;
  }
  
  duration_count--;
}

void next_note()
{
  if (note_n >= note_ct) {
    Serial.println("stopping");
    stop_tune();
    note_n = -1;
  } else {
    if (note_rep == 0) {
      Serial.print("end repeated: "); Serial.println(note_n);
      note_rep = -1;
      note_n += 2;
    } else if (tune_data[note_n] < 0) {

      if (note_rep < 0) {
        Serial.print("repeat set: "); Serial.println(note_rep);
        note_rep = tune_data[note_n + 1];
      } else {
        Serial.print("repeating: "); Serial.println(note_rep);
        note_rep--;
      }

      if (note_rep != 0)
        note_n = -1 - tune_data[note_n];
    } else {
      Serial.print("not rep: "); Serial.println(note_rep);
      play_note(tune_data[note_n], tune_data[note_n + 1]);
      note_n += 2;
    }
  }
}

bool tune_playing()
{
  delay(100);
  return note_n > -1;
}

ISR(TIMER1_COMPA_vect)
{
  pulse_spkr();

  if (duration_count <= 0) {
    next_note();
  }
}

void loop()
{
}
