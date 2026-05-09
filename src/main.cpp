#include <Arduino.h>
#define __Test__
// #define __AccessPoint__
#ifdef __Test__ 

// #include "tests/agvCtrlTest.h"
// #include "tests/agvTest.h"
#include "tests/agvTest2.h"
// #include "tests/dirCtrlTest.h"
// #include "tests/stepperLibTest.h"
// #include "tests/encoderLibTest.h"
// #include "tests/encoderClassTest.h"

// #include "hwtests/homePropeTest.h"
// #include "hwtests/odriveCmdTest.h"
// #include "hwtests/stepperPulseTest.h"
// #include "hwtests/stepperPulseTest2.h"
// #include "hwtests/encoderTest.h"
// #include "hwtests/timerTest.h"

#ifdef __AccessPoint__
    #include "tests/severTest.h"
#endif
#else
int main(){

    while (1){

    }
    return 0;
}
#endif
