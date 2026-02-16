    TCCR4A = 0; //Timer/Counter 
      //  • Bit 7:6 – COMnA1:0: Compare Output Mode for Channel A  // Set to zeros not connected to pin
      //  • Bit 5:4 – COMnB1:0: Compare Output Mode for Channel B  // Set to zeros not connected to pin
      //  • Bit 3:2 – COMnC1:0: Compare Output Mode for Channel C  // Set to zeros not connected to pin
      //  • Bit 1:0 – WGMn1:0: Waveform Generation Mode            // Mode 0, set to zeros, normal operation, OCnA OCnB OCnC disconnected
    TCCR4B = 0;
      //  • Bit 7 – ICNCn: Input Capture Noise Canceler            // Set to zero, off
      //  • Bit 6 – ICESn: Input Capture Edge Select               // Falling edge, whatever
      //  • Bit 5 – Reserved Bit                                   // Not used
      //  • Bit 4:3 – WGMn3:2: Waveform Generation Mode            // Mode 0, set to zeros, normal operation,OCnA OCnB OCnC disconnected
      //  • Bit 2:0 – CSn2:0: Clock Select                         // All zeros, No clock source (Timer stopped) (for now)
    TCCR4C = 0;
      //  • Bit 7 – FOCnA: Force Output Compare for Channel A      // Set to zero
      //  • Bit 6 – FOCnB: Force Output Compare for Channel B      // Set to zero
      //  • Bit 5 – FOCnC: Force Output Compare for Channel C      // Set to zero
      //  • Bit 4:0 – Reserved Bits                                // Not used
    TCCR4B |= (1<<WGM42);                   // WGM43-WGM42-WGM41-WGM40 = 0100 = Mode 4 = CTC (Clear Timer on Compare Match)
    TCCR4B |= (1<<CS40);                    // CS42-CS41-CS40 = 001 = Internal Clock Source no prescaler
    TIMSK4 |= (1<<OCIE4A);// Timer/Counter Interrupt Mask Register 4
      //  • Bit 5 – ICIEn: Timer/Countern, Input Capture Interrupt Enable
      //  • Bit 3 – OCIEnC: Timer/Countern, Output Compare C Match Interrupt Enable
      //  • Bit 2 – OCIEnB: Timer/Countern, Output Compare B Match Interrupt Enable
      //  • Bit 1 – OCIEnA: Timer/Countern, Output Compare A Match Interrupt Enable   // Set to 1 (this bit is OCIE4A)
      //  • Bit 0 – TOIEn: Timer/Countern, Overflow Interrupt Enable
  
    //TIMSK0 &= (0<<TOIE0); // TURN OFF TIMER0 and no more millis()! This makes the steps super smooth.
