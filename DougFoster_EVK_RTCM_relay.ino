/**
 * EVK RTCM relay.
 * 
 * Relays RTCM3 data from UART2 on a ZED F9P in a SparkFun EVK to an HC-12 radio.
 * 
 * @author   D. Foster <doug@dougfoster.me>.
 * @since    0.1.0 [2025-05-29-10:30pm] New.
 * @link     http://dougfoster.me.
 *
 * ===================================
 *      Comments.
 * ===================================
 * 
 * --- Description & operation. ---
 *     -- A SparkFun EVK is configured to operate as a GNSS base station. The base has a companion GNSS
 *        rover, in this case Ghost Rover. RTCM3 correction data is transmitted from the base to the rover via
 *        this sketch.
 *
 *        When the EVK is in base mode and a fix has been obtained, it will send an RTCM3 correction stream out
 *        over two lugs on a terminal block exposed at the back of the EVK. Inside the EVK enclosure, an
 *        ESP32-S3 MCU (SparkFun Thing Plus ESP32-S3 - identical to the one used by Ghost Rover) and an HC-12 RF
 *        radio (also matching one used by Ghost Rover) have been added. Both the MCU and HC-12 are powered over
 *        an I2C bus connection within the EV. The bus connection is used for power only and both MCUs are
 *        treated as "ships in the night" with no interaction between one another.
 *
 *        The ESP32-S3 MCU inside the EVK runs a simple serial relay sketch which reads data in byte-by-byte from
 *        one of the MCU's serial ports (which is wired to the EVK's terminal lugs (UART2 of the EVK's internal
 *        ZED-F9P). The sketch then relays the data byte-by-byte, out another serial port which is wired to the
 *        HC-12. The HC-12 transmits the serial data stream over RF to the rover.
 *
 *        An LED mounted on the EVK back panel blinks once for every 500 bytes of RTCM data sent.
 *
 * --- Major components. ---
 *     -- EVK   https://www.sparkfun.com/sparkfun-rtk-evk.html.
 *     -- MCU   https://www.sparkfun.com/sparkfun-thing-plus-esp32-s3.html.
 *     -- Radio (433.4-473.0 MHz, 100mW, U.FL): https://www.amazon.com/HiLetgo-Wireless-Replace-Bluetooth-Antenna/dp/B01MYTE1XR.
  *    -- Rover https://github.com/doug-foster/DougFoster_Ghost_Rover/.
 *
 * --- Other components. ---
 *     -- Radio antenna. --
 *        - UHF 400-960 MHz, BNC-M: https://www.amazon.com/dp/B07R4PGZK3.
 *        - cable (BNC-F bulkhead to U.FL, 8" RG178): https://www.amazon.com/dp/B098HX6NFH.
 *     -- Misc. --
 *        - LED cover (5mm LED bulb socket): https://www.amazon.com/dp/B07CQ6TH14.
 *
 * --- Misc. references. ---
 *     -- RTCM        https://www.use-snip.com/kb/knowledge-base/an-rtcm-message-cheat-sheet/.
 *     -- HC-12       https://www.elecrow.com/download/HC-12.pdf.
 *     -- EVK         https://docs.sparkfun.com/SparkFun_RTK_Everywhere_Firmware/menu_base/#rtcm-message-rates.
 *     -- SparkFun    https://learn.sparkfun.com/tutorials/tags/gnss.
 * 
 * --- Dev environment. ---
 *     -- Arduino IDE 2.3.6.
 *     -- Board: "Sparkfun ESP32-S3 Thing Plus" (~/Library/Arduino15/packages/esp32/hardware/esp32/3.2.0/boards.txt)
 *     -- VS Code 1.100.2 (Extensions: Better Comments, Bookmarks, C/C++, C/C++ Extension Pack, C/C++ Themes,
 *        CMake Tools, Dash, Diff Folders, Git Graph, GitHub Theme, GitLens, Markdown All in One, Serial Monitor,
 *        SFTP).
 *     -- GitHub repo: https://github.com/doug-foster/DougFoster_Ghost_Rover/.
 * 
 * --- Caveats. ---
 *     -- SoftwareSerial library is not supported on ESP32-S3 (does work on ESP32-C6).
 *
 * --- TODO: ---
 *     1. TBD
 *
 * --- Code flow. ---
 *     -- Include libraries.
 *     -- Global vars: define vars, set constants, prototypes.
 *     -- Functions: init, config, begin, start, check, callback, operation, tasks, test.
 *     -- Setup.
 *     -- Loop.
 */

// ===================================
//      Include libraries.
// ===================================

// --- Libraries. ---
#include <HardwareSerial.h>     // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.cpp.
#include <esp_system.h>         // https://github.com/pycom/esp-idf-2.0/blob/master/components/esp32/include/esp_system.h.

// ===================================
//      Global vars.
// ===================================

// -- Version. --
const char BUILD_DATE[]   = "2025-05-29-15:00";     // 24hr format, need to fit max (16) characters.
const char MAJOR_VERSION  = '0';
const char MINOR_VERSION  = '1';
const char PATCH_VERSION  = '0';

// -- Pin (pth) definitions. --
const uint8_t PTH_RTCM_TX = 4;      // ESP32-S3 Thing+ PTH 4 (UART1) <-> EVK UART2 TX (green wire) - RTCM.
const uint8_t PTH_RTCM_RX = 5;      // ESP32-S3 Thing+ PTH 5 (UART1) <-> EVK UART2 RX (yellow wire) - RTCM.
const uint8_t LED_RADIO   = 6;      // ESP32-S3 Thing+ PTH 6 <-> Blue LED (blue wire).
const uint8_t LED_SWITCH  = 10;     // ESP32-S3 Thing+ PTH 10 <-> Rocker switch (red wire).
const uint8_t PTH_SET     = 42;     // ESP32-S3 Thing+ PTH 42 (SET) <-> HC-12 SET (blue wire).
const uint8_t PTH_TX      = 43;     // ESP32-S3 Thing+ PTH 43 (UART2) <-> HC-12 RX (yellow wire).
const uint8_t PTH_RX      = 44;     // ESP32-S3 Thing+ PTH 44 (UART2) <-> HC-12 TX (white wire).

// -- Serial USB (monitor). --
const  uint8_t  NUM_COMMANDS     = 4;           // How many possible commands.
const  uint32_t SERIAL_MON_SPEED = 115200;      // Serial USB monitor speed.
       bool     testLEDr;                       // Test radio LED.
       bool     testRad;                        // Test radio.
       bool     debugRad;                       // Debug radio.
       bool     reset;                          // Reset MCU.
const  char     EXIT_TEST              = '!';   // Exit test mode.
const  char*    commands[NUM_COMMANDS] = {      // Valid commands. Point to array of C-strings.
                                         "testLEDr",
                                         "testRad",
                                         "debugRad",
                                         "reset"
};
       char     inputCharMon;                   // Monitor input read character.
       char     monitorCommand[11];             // Serial monitor command (C-string).
       char     radioCommand[11];               // serial (radio) test command (C-string).

// -- Serial1 (EVK-RTCM). --
const uint32_t       SERIAL1_SPEED = 38400;     // ZED-F9P default speed.
      char           charRTCM;                  // RTCM character.
      HardwareSerial serialEVK(1);              // UART1 object for ESP32-S3. Used for EVK.

// -- Serial2 (HC-12 radio). --
const uint32_t       SERIAL2_SPEED = 9600;      // HC-12 default speed.
      HardwareSerial serialRadio(2);            // UART2 object for ESP32-S3. Used for HC-12 radio.

// -- I2C. --
// Power only.

// -- LED display. --
const TickType_t LED_TIME_FLASH_ON  = 100/portTICK_PERIOD_MS;  // Time (ms).
const TickType_t LED_TIME_FLASH_OFF = 100/portTICK_PERIOD_MS;  // Time (ms).

// -- Task handles. --
TaskHandle_t radioRtcmLEDtaskHandle;            // Radio RTCM LED task handle.

// -- Operation. --

// -- Prototypes. --
void updateLED(char, char, char);              // Eliminate compiler scope error due to definition order.

// -- Test. --

// ===================================
//             Functions
// ===================================

// --- Init. ---

/**
 * ------------------------------------------------
 *      Initialize global vars.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void initVars() {

    // -- Serial USB (monitor). --
    Serial.print("Initialize global vars");
    memset(monitorCommand, '\0', sizeof(monitorCommand));
    memset(radioCommand, '\0', sizeof(radioCommand));

    // -- Serial1 (EVK-RTCM). --
    charRTCM = '\0';

    // -- Operation. --
    testLEDr  = false;
    testRad   = false;
    debugRad  = false;
    reset     = false;
}

// --- Config. ---

/**
 * ------------------------------------------------
 *      Initialize pin modes & pin values.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void configPins() {

    // -- Initialize pin modes. --
    Serial.print("Config pins");
    pinMode(LED_RADIO,   OUTPUT);

    // -- Initialize pin values. --
    digitalWrite(LED_RADIO, LOW);
    Serial.println(".");
}

// --- Begin. ---

/**
 * ------------------------------------------------
 *      Begin serial (USB) for monitor.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void beginSerialUSB() {

    // -- Begin USB interface. --
    Serial.begin(SERIAL_MON_SPEED);
    Serial.println("\nBegin serial monitor(USB) @ 115,200 bps.");
}

/**
 * ------------------------------------------------
 *      Begin serial1 (UART1) for EVK-RTCM.
 * ------------------------------------------------
 * 
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void beginSerial1EVK() {

    // -- Begin serial1 interface. --
    Serial.print("Begin serial1 (UART1) for EVK-RTCM @ 38,400 bps");
    serialEVK.begin(SERIAL1_SPEED, SERIAL_8N1, PTH_RTCM_TX, PTH_RTCM_RX);     // UART1 object. TX<->TX, RX<->RX.
    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Begin serial2 (UART2) for HC-12 radio.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void beginSerial2Radio() {

    // -- Begin serial2 interface. --
    Serial.print("Begin serial2 (UART2) for HC-12 radio @ 9,600 bps");
    serialRadio.begin(SERIAL2_SPEED, SERIAL_8N1, PTH_RX, PTH_TX);   // UART2 object. TX<->RX, RX<->TX.
    Serial.println(".");

    // -- Set pin. --
    Serial.print("Radio config pin set high (transparent mode)");
    pinMode(PTH_SET, OUTPUT);                                       // Prepare pin for HC-12 AT command set mode.
    digitalWrite(PTH_SET, HIGH);                                    // Set pin HIGH for transparent.
    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Begin I2C interface.
 * ------------------------------------------------
 *
 * -- Power only
 */

// --- Start. ---

/**
 * ------------------------------------------------
 *      Start tasks.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 * @see    Global vars: Task handles.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate.
 */
void startTasks() {

    // -- Create Tasks. --
    xTaskCreate(radioRtcmLEDtask,    "radio_RTCM_LED_task",       2048, NULL, 2, &radioRtcmLEDtaskHandle);

    // -- Suspend tasks. --
    vTaskSuspend(radioRtcmLEDtaskHandle);

    // -- Print status. --
    Serial.println("Create/suspend LED task: radio RTCM.");
}

/**
 * ------------------------------------------------
 *      Start UI.
 * ------------------------------------------------
 *
 * Last step in setup before loop().
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void startUI() {

    // -- Verify LEDs.
    Serial.print("Verify LEDs");
    updateLED('1');        // LED on.
    delay(1000);            // Pause.
    updateLED('0');        // LED off.
    Serial.println(".");

    // -- Loop. --
    Serial.println("Starting loop.");

    // -- Display serial (USB) startup message. --
    Serial.printf("\nEVK RTCM relay - Version %c.%c.%c @ %s.\n\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION, BUILD_DATE);
}

// --- Check. ---

/**
 * ------------------------------------------------
 *      Check for serial (USB) input.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 */
void checkSerialUSB(char print = ' ') {

    // -- Local vars. --
    static uint8_t posnMon = 0;         // Persistant input position for USB serial monitor command.
    static uint8_t posnRad = 0;         // Persistant input position for radio test command.
    uint8_t whichMonitorCommand;        // Which command was entered from the USB serial monitor.

    // -- Print valid commands. --
    if(print == 'p') {
        Serial.print("\nValid commands: ");
        for (size_t i = 0; i < NUM_COMMANDS-1; i++) {
            if ((i != 0) && (i % 7 == 0)) {
                Serial.println();
            }
            Serial.printf("%s, ", commands[i]);
        }
        Serial.printf("%s.\n! to quit.\n\n", commands[NUM_COMMANDS-1]);
        return;                         // Done for now.
    }

    // -- Read serial input. --
    while (Serial.available() > 0) {    // Input from serial monitor.
        inputCharMon = Serial.read();   // Read byte in from USB serial.
        if (inputCharMon != '\n' && (posnMon < (sizeof(monitorCommand) - 1))) {    // Are we done?
            monitorCommand[posnMon] = inputCharMon;     // Not done yet, add char to command.
            posnMon++;                                  // Increment command buffer posn.
        } else {
            monitorCommand[posnMon] = '\0';             // We're done reading, treat command[] as C-string.
            posnMon = 0;                                // Reset command buffer position.

        // - Which command? -
        if(*monitorCommand == EXIT_TEST) {              // Reset debug flags & return.
            debugRad   = false;
            Serial.println();
            return;
        }
        whichMonitorCommand = 99;       // Which command was entered. Assume invalid until validated.
            for (size_t i = 0; i < NUM_COMMANDS; i++) {
            if(strcmp(monitorCommand, commands[i]) == 0) {      // Compare C-strings.
                whichMonitorCommand = i;
                break;
            }
        }

        // - Valid command? -
        if(whichMonitorCommand < 99) {   
            // Print the validated command.
            // Toggle command flag & print new state.
            switch (whichMonitorCommand) {
                    case 0:
                        testLEDr = (testLEDr == true) ? false : true;       // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (testLEDr  ? "enabled." : "disabled."));
                        break;
                    case 1:
                        testRad = (testRad == true) ? false : true;         // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (testRad  ? "enabled." : "disabled."));
                        break;
                    case 2:
                        debugRad = (debugRad == true) ? false : true;       // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (debugRad ? "enabled." : "disabled."));
                        break;
                    case 3:
                        reset = (reset == true) ? false : true;             // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (reset       ? "enabled." : "disabled."));
                        break;
                };
                monitorCommand[0] = '\0';                                   // Ready for the next time.
            } else {

                // Invalid command.
                Serial.printf("\n%s is not a valid command. \n", monitorCommand);

                // Display valid serial Monitor commands.
                checkSerialUSB('p');
            }

            // - Reset MCU. -
            if (reset) {
                Serial.println("Restarting ...");
                esp_restart();
            }

            // - Test radio LED. -
            if (testLEDr) {

                // Display instructions.
                Serial.printf("Valid options: 0(off), 1(on), 2(active). %c to quit.\n", EXIT_TEST);

                // Loop.
                while (true) {                                  // Infinite loop.
                    if (Serial.available() > 0) {
                        inputCharMon = Serial.read();           // Read input from serial monitor.
                        Serial.read();                          // Discard newline.
                        switch (inputCharMon) {
                            case EXIT_TEST:                     // All done.
                                Serial.println("testLEDr disabled.");
                                testLEDr = false;               // Clear test flag.
                                return;                         // Exit test mode.
                            case '0':                           // Radio LED - off.
                                Serial.printf("%c - radio LED off.\n", inputCharMon);
                                updateLED('0');
                                break;
                            case '1':                           // BLE LED - on.
                                Serial.printf("%c - radio LED on.\n", inputCharMon);
                                updateLED('1');
                                break;
                            case '2':                           // BLE LED - active.
                                Serial.printf("%c - radio LED active - 5 cycles.\n", inputCharMon);
                                for (size_t i = 0; i < 5; i++) {
                                    updateLED('2');
                                    Serial.printf("Blink %i\n", i+1);
                                    delay(1000);
                                }
                                Serial.println();
                                break;
                            default:
                                Serial.printf("%c to quit. Valid options: 0(off), 1(on), 2(active).\n", EXIT_TEST);
                        }
                    }
                }
            }

            // - Test radio. -
            if (testRad) {

                // HC-12 AT command mode.
                radioCommand[0] = '\0';             // Reset read buffer.
                posnRad = 0;
                digitalWrite(PTH_SET, LOW);
                serialRadio.read();                 // Garbage first character.
                updateLED('0');                     // Radio LED off - AT command mode.

                // Display instructions.
                Serial.println("\nHC-12 command mode enabled (! to exit)");
                Serial.println("  AT, AT+Bxxxx, AT+Cxxx, AT+FUx, AT+Px,");
                Serial.println("  AT+Ry (AT+RB, AT+RC, AT+RF, AT+RP, AT+RX),");
                Serial.println("  (y = B=baudrate, C=channel, F=mode, P=power),");
                Serial.println("  AT+Udps, AT+V, AT+SLEEP, AT+DEFAULT, AT+UPDATE.");
                Serial.println("  https://www.datsi.fi.upm.es/docencia/DMC/HC-12_v2.3A.pdf\n");

                // Loop.
                while (true) {                                              // Infinite loop.
                    if (Serial.available() > 0) {
                        inputCharMon = Serial.read();                       // Read input from serial monitor.
                        if(inputCharMon == EXIT_TEST) {                     // All done?
                            Serial.println("HC-12 command mode disabled.\n");
                            digitalWrite(PTH_SET, HIGH);
                            Serial.read();                                  // Clear the newline.
                            testRad = false;                                // Clear test flag.
                            return;                                         // Exit test mode.
                        } else {
                            inputCharMon = toupper(inputCharMon);           // Convert char to upper case.
                        }
                        switch (inputCharMon) {
                            case '\n':                                      // Interact with HC-12.
                                serialRadio.write(radioCommand);            // Write command to HC-12.
                                Serial.println("");
                                delay(200);                                 // Allow HC-12 to process command & respond.
                                while (serialRadio.available() > 0) {       // Read response from HC-12.
                                    inputCharMon = '\0';
                                    inputCharMon = serialRadio.read();
                                    if ((255 != (int) inputCharMon) && (posnRad > 0)) {    // Ignore first garbage character.
                                        Serial.print(inputCharMon);         // Echo character to serial monitor.
                                    }
                                }
                                radioCommand[0] = '\0';                     // Reset read buffer.
                                posnRad=0;
                                break;
                            default:                                        // Echo & save input character.
                                if (255 != (int) inputCharMon) {
                                    Serial.print(inputCharMon);
                                    radioCommand[posnRad] = inputCharMon;   // Add character to command buffer.
                                    posnRad++;
                                }
                        }
                    }
                }
            }

            // - Other. -
            // @see checkRTCMtoRadio().
        }
    }
}

/**
 * ------------------------------------------------
 *      Check serial1 (EVK-RTCM). Send to serial2 (HC-12 radio).
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-05-29-10:30pm] New.
 * @see    beginSerial1EVK(), beginSerial2Radio().
 * @link   https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3/blob/main/examples/ZED-F9P/Example3_StartRTCMBase/Example3_StartRTCMBase.ino.
 */
void checkRTCMtoRadio() {

    // -- Local vars. --
          uint16_t count    = 0;
    const uint16_t maxCount = 500;

    // -- Read serial1 (EVK-RTCM) input. Send to serial2 (HC-12 radio). --
    if (serialEVK.available() > 0) {        // serial1 (EVK-RTCM) data to read?

        // - Read from serial1 (EVK-RTCM). -
        charRTCM = serialEVK.read();        // Read a character @ SERIAL1_SPEED.

        // - Write to serial2 (HC-12 radio). -
        serialRadio.write(charRTCM);        // Write a character @ SERIAL2_SPEED.

        // - Debug. -
        if (debugRad) {
            Serial.printf("%x\n", charRTCM);
        }

        // - Update UI. -
        if (count == maxCount) {
            updateLED('2');                 // Radio LED - active.
            count = 0;
        } else {
            count++;
        }
    }
}

/**
 * ------------------------------------------------
 *      Toggle LED.
 * ------------------------------------------------
 *
 * @param  char ledR Radio LED.
 * @return void No output is returned.
 * @since  0.3.3 [2025-05-29-10:00pm] New.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/07-vTaskResume.
 */
void updateLED(char ledR) {

    // --- Radio LED. ---
    switch (ledR) {
        case '0':
            digitalWrite(LED_RADIO, LOW);               // LED off.
            break;
        case '1':
            digitalWrite(LED_RADIO, HIGH);              // LED on.
            break;
        case '2':
            vTaskResume(radioRtcmLEDtaskHandle);        // Resume task.
            break;
    }
}

// --- Callback. ---

// --- Operation. ---

// --- Tasks. ---

/**
 * ------------------------------------------------
 *      Radio LED - active task.
 * ------------------------------------------------
 *
 * @param  void * pvParameters Pointer to task parameters.
 * @return void No output is returned.
 * @since  0.3.3 [2025-05-29-10:00pm] New.
 * @see    startTasks().
 * @link   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 */
void radioRtcmLEDtask(void * pvParameters) {
    while(true) {
        digitalWrite(LED_RADIO, HIGH);                      // LED on.
        vTaskDelay(LED_TIME_FLASH_ON);                      // LED remains on (ms).
        digitalWrite(LED_RADIO, LOW);                       // LED off.
        vTaskSuspend(NULL);                                 // Suspend task.
    }
}

// --- Test. ---

// ===================================
//               Setup.
// ===================================

void setup() {
    beginSerialUSB();               // Begin serial (USB) for monitor.
    initVars();                     // Initialize global vars.
    configPins();                   // Initialize pin modes & pin values.
    beginSerial1EVK();              // Begin serial1 (UART1) for EVK-RTCM.
    beginSerial2Radio();            // Begin serial2 (UART2) for HC-12 radio.
    startTasks();                   // Start tasks.
    startUI();                      // Start UI.
}                                               

// =================================== 
//               Loop.
// ===================================

void loop() {
    checkSerialUSB();               // Check for serial (USB) input.
    checkRTCMtoRadio();             // Check serial1 (EVK-RTCM). Send to serial2 (HC-12 radio).
}
