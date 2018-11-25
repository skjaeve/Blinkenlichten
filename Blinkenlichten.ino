/* A simple board with four switches, 16 regular LEDs and a PWM board to run the LEDs. */

/* Flip a switch:
 *  Board wakes up from sleep, or restarts its running sequence
 *  The four LEDs associated with each switch turns on if the switch is on, otherwise off.
 *  
 *  After a while the LEDs fade out.
 *  
 *  A number of LEDs defined by taking the switch settings to be a binary number, plus 1, is chosen to be lit.
 *  Which LEDs to light is chosen randomly. Each LED goes through a "truncgauss" fadein-fadeout sequence. 
 *  The duration of each fi-fo sequence is chosen randomly within some range.
 *  When a LED fades out, a new LED is chosen randomly to fade in/out with a new duration. 
 *  This continues until some time limit, after which all active LEDs complete their running fi-fo and the 
 *  LED board goes black.
 *  
 *  Board now enters a sleep-check-sleep-check loop state, waiting for something to happen or the battery to run out. 
 *  If a switch is flipped at any point in the whole sequence, an interrupt triggers, and the sequence starts from scratch.
 *  
 *  May add random flashes of light to sleep-check cycle.
 */
/* Version fire: erstatt blinke-rulling med:
  *  tilsvarar brytarinnstilling omsett til binærtal
 *  X lamper lyser opp og sløkker, lysstyrken følgjer ein gammakorrigert trunkert Gauss-kurve. Lengda på syklus varierer (velg tilfeldig)
 *  Når ei lampe gjennomfører syklus blir ei ny lampe starta i ny syklus - kan vera same eller anna, lampe og sykluslengde vert plukka tilfeldig for kvar gong
 *  Avslutning: etter ei viss tid sluttar brettet å starta nye gauss-syklusar, dei køyrande avsluttar og lampene går gradvis i svart
 *  
 *  Korleis  - funksjon eller tabell? Testing med gamma tyder på at funksjon tar ~2 gang så lang tid som PROGMEM-tabell men det er truleg ikkje noko problem
 *  å berre kompensera med å dobla tidssteget i funksjonen jf. med tabellen. Kan uansett finstemma funksjonen sånn etterkvart.
 */
 
 /****************************************************/
#include <LowPower.h>
// See https://github.com/rocketscream/Low-Power/issues/45 for possibly necessary hack to make this run on ATmega168P
// see also http://forum.arduino.cc/index.php?topic=248690.0
// datasheet https://www.mouser.com/pdfdocs/Gravitech_ATMEGA328_datasheet.pdf
#include "truncgauss.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PinChangeInterrupt.h>

#ifndef DEBUG
#define DEBUG 0
#endif

// Corresponds to digitalRead() returns from pins in INPUT_PULLUP mode, which the switches should be
#define SWITCH_ON (LOW)
#define SWITCH_OFF (HIGH)

// There are multiple stages
// Keep track of where we are in each stage
int numstages = 0;
volatile int STAGE = 0;
const int LIGHTUP_STAGE = numstages++;
const int WAIT_STAGE = numstages++;
const int FADE_OUT_STAGE = numstages++;
const int PREPARE_GAUSSIAN_STAGE = numstages++;
const int GAUSSIAN_STAGE = numstages++;
const int SLEEP_STAGE = numstages++;
const int FLASH_SOME_LIGHTS_STAGE = numstages++;
volatile bool WAS_INTERRUPTED = false;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const byte switchPins[4] = {8, 9, 10, 11};
// For timekeeping without delay()
unsigned long lastSwitchFlippedMillis = 0;
unsigned long stageEndMillis = 0;

unsigned long previousLedChangeMillis;        // will store last time any LED was updated

unsigned short int ledsToLight[16];
// ledLevels contains LINEAR led value, before ledGamma() function // or not, ledLEvel shoudl come from truncgauss() now
short int ledLevels[16];
unsigned short int ledsChosen = 0;
bool ledTimeout = true;

uint8_t switchNumber = 0;

// Used for the gaussian sequences
unsigned short int gaussLedTotalDuration[16];
unsigned short int gaussLedStartMillis[16];


void switchFlipped(void) {
    WAS_INTERRUPTED = true;
}

void setup() {

    // disable ADC
    ADCSRA = 0;

#if DEBUG
    Serial.begin(9600);
    Serial.println("DEBUG: Initialize 16-LED switchboard");
#endif


    pwm.begin();
    pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

    // Switches read HIGH when off
    for (int i = 0; i < 4; i++) {
        pinMode(switchPins[i], INPUT_PULLUP);
    }
    lastSwitchFlippedMillis = millis();
    previousLedChangeMillis = millis();
 
    for (int i = 0; i < 4; i++) {
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(switchPins[i]), switchFlipped, CHANGE);
#if DEBUG
        Serial.print("DEBUG: Interrupt added: pin ");
        Serial.println(switchPins[i]);
#endif
    }
#if DEBUG
    Serial.println("DEBUG: Setup done");
#endif
    
}

void loop() {

    if (WAS_INTERRUPTED == true) {
#if DEBUG
        Serial.println("DEBUG: INTERRUPT");
#endif
        WAS_INTERRUPTED = false;
        STAGE = LIGHTUP_STAGE;
        lastSwitchFlippedMillis = millis();
        ledsChosen = 0;
    }
    if (STAGE == LIGHTUP_STAGE) {
#if DEBUG
        Serial.println("DEBUG: LIGHTUP_STAGE");
#endif
        // This switches on/off the groups of four LEDs above each switch.
        int ledGroupBase[4] = {6, 7, 14, 15}; // LEDs associated with switch 0, or was it 3?
        for (int switchPinCt = 0; switchPinCt < 4; switchPinCt++) {
            bool pinState = digitalRead(switchPins[switchPinCt]);
            for (int ledInGroup = 0; ledInGroup < 4; ledInGroup++) {
                int ledNum = ledGroupBase[ledInGroup] - switchPinCt * 2;
                int ledLevel = 4096 * !pinState; // Turn off led if pin is HIGH else on if LOW
                pwm.setPin(ledNum, ledGamma(ledLevel));
                ledLevels[ledNum] = ledLevel;
            }
        }

        stageEndMillis = millis();
        STAGE++;

    } else if (STAGE == WAIT_STAGE) {
#if DEBUG
        Serial.println("DEBUG: WAIT_STAGE");
#endif
        if (millis() - stageEndMillis < 2000) {
            LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_ON, TWI_ON);
        } else {
            stageEndMillis = millis();
            STAGE++;
        }

    } else if (STAGE == FADE_OUT_STAGE) {
#if DEBUG
        Serial.println("DEBUG: FADE_OUT_STAGE");
#endif
        // Return all LEDs to zero, but without blinking them
        // Compute linear progress from 4096 to 0 over duration
        short int fade_duration = 5000; // How long the fade-out should take
        short int fadeout_progress = millis() - stageEndMillis; // How far into the fade-out

        short int ledLevel = map(fadeout_progress, 0, fade_duration, 4096, 0);
        for (uint8_t led = 0; led < 16; led++) {
            if (ledLevel < ledLevels[led]) {
                pwm.setPin(led, ledGamma(ledLevel));
                ledLevels[led] = ledLevel;
            }
        }
        if (fadeout_progress >= fade_duration) {
            stageEndMillis = millis();
            STAGE++;
        }

    } else if (STAGE == PREPARE_GAUSSIAN_STAGE) {
#if DEBUG
        Serial.println("DEBUG: PREPARE_GAUSSIAN_STAGE");
#endif                

        switchNumber = 0;
        for (uint8_t i = 0; i < 4; i++) {
            switchNumber += ((int) !digitalRead(switchPins[i])) << i;
        }
        switchNumber++; // Allow all 16 leds to be lit (shift from [0, 15] lit to [1, 16])
           /* Init ledsToLight array */
        for (int led = 0; led < 16; led++) {
            ledsToLight[led] = 0;
            pwm.setPin(led, ledGamma(0));
        }
        ledsChosen = 0;

        stageEndMillis = millis();
        STAGE++;

    } else if (STAGE == GAUSSIAN_STAGE) {
#if DEBUG
        Serial.print("DEBUG: GAUSSIAN_STAGE ");
        Serial.print(ledsChosen);
        Serial.print(" LEDS CHOSEN SWITCH SETTING ");
        Serial.println(switchNumber);

#endif
        unsigned int gaussians_duration = 20000; // After this time has elapsed, no more gaussians are started
        unsigned int max_gaussian_duration = 5000;
        unsigned int min_gaussian_duration = 1000;
        
        // Choose new LEDs and durations if needed
        while ((ledsChosen < switchNumber) && ((millis() - stageEndMillis) < gaussians_duration)) {
            int led = random(16);
            if (ledsToLight[led] == 0) {
                ledsToLight[led] = 1;
                ledsChosen++;
                gaussLedTotalDuration[led] = random(min_gaussian_duration, max_gaussian_duration+1);
                gaussLedStartMillis[led] = millis();
            }
        }

        for (int led = 0; led < 16; led++) {
            short int ledLevel = 0;
            if (ledsToLight[led] == 1) {
                int total_duration = gaussLedTotalDuration[led];
                int time_elapsed = millis() - gaussLedStartMillis[led];
                if (time_elapsed > total_duration) {
                    ledsChosen--;
                    ledsToLight[led] = 0;
                    ledLevel = 0;
                } else {
                    ledLevel = truncgauss(time_elapsed, total_duration);
                }
                pwm.setPin(led, ledGamma(ledLevel));
            } else {
                pwm.setPin(led, ledGamma(0));
            }
        }

        if (((millis() - stageEndMillis) > gaussians_duration) && (ledsChosen == 0)) {
            stageEndMillis = millis();
            STAGE++;
        }
    } else if (STAGE == SLEEP_STAGE) {
#if DEBUG
        Serial.println("DEBUG: SLEEP_STAGE");
#endif
         /* 
          *  Sleeping doesn't actually do much power saving until I build a custom 
          * Atmega328-breakout with efficient buck converter and other power-saving
          * thingies such as a relay/mosfet switching on/off power to the leds and 
          * PWM board. With the standard Arduino (clone) setup most of the power is
          * wasted while idling. My setup runs for about four days off six 2450 mAh
          * 1.2V NIMH-batteries (in series).
          */
 
#if DEBUG
        Serial.println("DEBUG: yawn");
        delay(500);
#else
        LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
#endif
        //FIXME code to have a one-in-a-million chance of skipping to next stage after 30min
        if ((millis() - stageEndMillis) > (30 * 60 * 1000L) && random(1000000) == 42) {
            stageEndMillis = millis();
            STAGE++;
        }

    } else if (STAGE == FLASH_SOME_LIGHTS_STAGE) {
#if DEBUG
        Serial.println("DEBUG: FLASH_SOME_LIGHTS_STAGE");
#endif
        STAGE = PREPARE_GAUSSIAN_STAGE;
        stageEndMillis = millis();
    }
}
