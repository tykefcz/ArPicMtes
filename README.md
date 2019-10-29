# ArPicMtes - Arduino AVR Pin Change Interrupt library
Library for save timestamp on pin change - for ultrasonic or buttons.

## Functions
- <code>pcAttachPin(#);pcDetachPin(#)</code>
- <code>while(isPcEvent()) {<br/>if (pcEventPin() == ECHO) { // Edge <br/>    if (pcEventRiseEdge()) <br/>      starttime=pcEventStamp(); <br/>    else <br/>      timeToEcho = ticksToMicros(pcEventStamp() - starttime); <br/>  } else if (pcEventPin() == ....<br/>....<br/>  pcEventPop();<br/>}</code>

