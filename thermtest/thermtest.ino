/*

Thermtest - Harness used to test various thermistor reading strategies using Teensy 3.2 ADC

Copyright (c) 2018 Joel Clark
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIEDi
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <ADC.h>

#define PING_TIMEOUT 5000
#define SLOW_PIN_COUNT 7

float calculate(uint16_t steps);

ADC *adc = new ADC(); // adc object
elapsedMillis timeSinceLastPing = 99999;

uint8_t slow_pins[7];
uint8_t slow_pin_index = 0;
uint8_t fast_pin;

void setup()
{
    fast_pin = A2;

    slow_pins[0] = A3;
    slow_pins[1] = A4;
    slow_pins[2] = A5;
    slow_pins[3] = A6;
    slow_pins[4] = A7;
    slow_pins[5] = A8;
    slow_pins[6] = A9;

    Serial.begin(115200);

    pinMode(fast_pin, INPUT);

    for (uint8_t i = 0; i < SLOW_PIN_COUNT; i++)
    {
        pinMode(i, INPUT);
    }

    adc->setReference(ADC_REFERENCE::REF_3V3);

    adc->setAveraging(1);
    adc->setResolution(12);
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
}

void loop()
{
    while (Serial.available() > 0)
    {
        char incoming = Serial.read();
        timeSinceLastPing = 0;
    }

    if (timeSinceLastPing < PING_TIMEOUT)
    {
        measure(fast_pin, 25);
        measure(slow_pins[slow_pin_index++], 25);

        if (slow_pin_index >= SLOW_PIN_COUNT)
        {
            slow_pin_index = 0;
        }
    }
    else
    {
        slow_pin_index = 0;
    }
}

void measure(uint8_t pin, int waitTime)
{
    elapsedMillis timeTaken = 0;
    int count = 0;
    long output = 0;

    while (timeTaken < waitTime)
    {
        count++;
        output += adc->analogRead(pin);
    }

    int average = output / count;

    Serial.print(pin);
    Serial.print(":");
    Serial.print(average);
    Serial.print("#");
    Serial.print(count);
    Serial.print("/");
    Serial.print(timeTaken);
    Serial.print("/");
    Serial.print(timeSinceLastPing);
    Serial.println();
}
