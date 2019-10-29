# ArPicMtes - Arduino AVR Pin Change Interrupt library
Library for save timestamp on pin change - for ultrasonic or buttons.

## Functions
- <code>pcAttachPin(#);pcDetachPin(#)</code>
- <code>while(isPcEvent()) {</code><br/><code>if (pcEventPin() == ECHO) { // Edge </code><br/><code>    if (pcEventRiseEdge()) </code><br/><code>      starttime=pcEventStamp(); </code><br/><code>    else </code><br/><code>      timeToEcho = ticksToMicros(pcEventStamp() - starttime); </code><br/><code>  } else if (pcEventPin() == ....</code><br/><code>....</code><br/><code>  pcEventPop();</code><br/><code>}</code>

