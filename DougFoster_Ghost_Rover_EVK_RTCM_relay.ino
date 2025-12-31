/**
 * **********************************************************************
 *      Ghost Rover 3 - RTCM relay from EVK ZED-F9P to HC-12 serial RF radio.
 * **********************************************************************
 * 
 * @author   D. Foster <doug@dougfoster.me>.
 * @since    3.0.10 [2025-12-30-06:00pm] New.
 * @see      https://github.com/doug-foster/DougFoster_Ghost_Rover.
 * @see      https://github.com/doug-foster/DougFoster_Ghost_Rover_BT_relay.
 * @see      https://github.com/doug-foster/DougFoster_Ghost_Rover_EVK_RTCM_relay.
 * @link     http://dougfoster.me.
 */

/**
 * ----------------------------------------------------------------------------
 *                          Comments.
 * ----------------------------------------------------------------------------
 * 
 * --- Description & operation. ---
 *     -- A SparkFun EVK is configured to operate as an assisted base GNSS station. The base has a companion GNSS
 *        rover - Ghost Rover. This sketch transmits RTCM3 correction data from the base to the rover.
 * 
 *        Inside the EVK enclosure, a SparkFun Thing Plus ESP32-C6 and an HC-12 RF radio (powered by a Qwiic I2C bus
 *        connection inside the EVK - power only, no data) have been added.
 *
 *        When the EVK is in base mode and a fix has been obtained, the EVK's ZED-F9P GNSS processor (UART2) will send
 *        out a serial stream of RTCM3 correction data via a terminal block mounted on the back panel of the EVK.
 *        The ESP32-C6 is connected to the TX2 lug on this terminal block.
 *
 *        In the loop(), data is read byte-by-byte from the ZED-F9P UART2 by checkRTCMtoRadio() and transfered to
 *        the HC-12. The HC-12 transmits the serial RTCM3 stream over RF to the rover's receiving HC-12.
 *
 *        An LED mounted on the EVK back panel blinks once for every RTCM3 sentence transmitted.
 *
 * --- Major components. ---
 *     -- EVK   https://www.sparkfun.com/sparkfun-rtk-evk.html.
 *     -- MCU   https://www.sparkfun.com/sparkfun-thing-plus-esp32-c6.html.
 *     -- Radio (433.4-473.0 MHz, 100mW, U.FL) https://www.amazon.com/HiLetgo-Wireless-Replace-Bluetooth-Antenna/dp/B01MYTE1XR.
 *     -- Rover https://github.com/doug-foster/DougFoster_Ghost_Rover/.
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
 *     -- IDE        VS Code & Arduino Maker Workshop 1.0.8 extension (uses Arduino CLI 1.4).
 * 
 * --- Caveats. ---
 * --- TODO: ---
 * --- Code organization. ---
 *     -- Include libraries.
 *     -- Global vars.
 *     -- Setup functions.
 *     -- Task functions.
 *     -- Loop functions.
 *     -- Setup.
 *     -- Loop.
 */

/**
 * ============================================================================
 *                          Include libraries.
 * ============================================================================
 *
 * @since                   3.0.9 [2025-12-17-06:00pm] New.
 * @link  Arduino           https://docs.arduino.cc/libraries/.
 * @link  ESP32             https://docs.espressif.com/projects/arduino-esp32/en/latest/libraries.html.
 */

// --- Core. ---
#include <Arduino.h>            // https://github.com/espressif/arduino-esp32.
#include <esp_system.h>         // https://github.com/pycom/esp-idf-2.0/blob/master/components/esp32/include/esp_system.h.
#include <esp_chip_info.h>      // https://github.com/pycom/pycom-esp-idf.

// --- Additional. ---

/**
 * ============================================================================
 *                          Global vars.
 * ============================================================================
 *
 * @since  3.0.9 [2025-12-15-06:30pm] New.
 */

// --- Pin asignments. ---

// -- Serial0 (UART0). --
const uint8_t RTCM_IN  = 5;                 // ZED TX2 (RTCM) {green wire}.
const uint8_t RTCM_OUT = 4;                 // ZED RX2 (not used) <- RTCM {yellow wire}.

// -- Serial1 (UART1). --   
const uint8_t HC12_TX  = 16;                // HC-12 TXD {yellow wire}.
const uint8_t HC12_RX  = 17;                // HC-12 RXD {white wire}.
const uint8_t HC12_SET =  2;                // HC-12 SET {blue wire}.

// -- LED. --
const uint8_t LED_RADIO = 3;                // Red LED {blue wire}.

// --- Serial. ---
const uint32_t SERIAL_USB_SPEED = 115200;   // Serial USB speed.
const uint32_t SERIAL0_SPEED    = 57600;    // ZED default speed.
const uint32_t SERIAL1_SPEED    = 9600;     // HC-12 default speed.
      char monitorChar;                     // Monitor i/o character.  // ToDo.
      char serialChar;                      // Serial i/o character.
      char rtcmSentence[300];               // RTCM3 sentence buffer.

// --- I2C. ---
// Power only.

// --- Timing. ---
const TickType_t LED_TIME_FLASH_ON = 100/portTICK_PERIOD_MS;  // Time (ms).

// --- Task handles. ---
TaskHandle_t radioRtcmLEDtaskHandle;            // Radio RTCM LED task handle.

// --- Operation. ---
const uint8_t NUM_COMMANDS           = 4;       // How many possible commands.
const char    EXIT_TEST              = '!';     // Exit test mode.
const char*   COMMANDS[NUM_COMMANDS] = {        // Valid commands. Point to array of C-strings.
                                         "testLEDr",
                                         "testRad",
                                         "debugRad",
                                         "reset"
};
      char    monitorCommand[11];               // Serial monitor command (C-string). // ToDo.
      char    radioCommand[11];                 // serial (radio) test command (C-string). // ToDo.
      bool    testLEDr;                         // Test radio LED.
      bool    testRad;                          // Test radio.
      bool    debugRad;                         // Debug radio.
      bool    reset;                            // Reset MCU.
      bool    inLoop;                           // In loop indicator.
esp_chip_info_t chip_info;                      // Chip info.

// --- Version. ---
const char BUILD_DATE[]  = "[2025-12-30-06:30pm]";
const char MAJOR_VERSION = '3';
const char MINOR_VERSION = '0';
const char PATCH_VERSION = '10';
const char NAME[]        = "Ghost Rover 3 - RTCM Relay";

// --- Declaration. ---
void updateLED(char);

// --- Test. ---

/**
 * ============================================================================
 *                          Setup functions.
 * ============================================================================
 */

/**
 * ------------------------------------------------
 *      Display build & processor info.
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.10 [2025-12-30-02:00pm].
 * @see    setup().
 */
void showBuild() {
    Serial.begin(SERIAL_USB_SPEED);
    Serial.printf("\n%s, Version: %c.%c.%c, Build date: %s\n", NAME, MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION, BUILD_DATE);
    esp_chip_info(&chip_info);
    Serial.printf("Using %s, Rev %d, %d core(s), ID (MAC) %012llX.\n",
    ESP.getChipModel(), chip_info.revision, chip_info.cores, ESP.getEfuseMac());
}

/**
 * ------------------------------------------------
 *      Start serial interfaces.
 * ------------------------------------------------
 * 
 * @return void No output is returned.
 * @since  3.0.9  [2025-12-14-02:00pm] New.
 * @since  3.0.10 [2025-12-30-02:00pm] Add Serial USB.
 * @see    setup().
 * @link   https://randomnerdtutorials.com/esp32-uart-communication-serial-arduino/#esp32-custom-uart-pins.
 */
void startSerial() {

    // --- Serial interface. ---
    Serial.println("\nSetup() started.");
    Serial.printf("Serial (USB) started @ %i bps.\n", SERIAL_USB_SPEED);

    // --- Serial0 interface. ---
    Serial.printf("Serial0 (ZED) started @ %i bps", SERIAL0_SPEED);
    Serial0.begin(SERIAL0_SPEED, SERIAL_8N1, RTCM_IN, RTCM_OUT);     // UART0 object. RX, TX.
    Serial.println(".");

    // --- Serial1 interface. ---
    Serial.printf("Serial1 (HC-12) started @ %i bps", SERIAL1_SPEED);
    Serial1.begin(SERIAL1_SPEED, SERIAL_8N1, HC12_RX, HC12_TX);     // UART1 object. RX, TX.
    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Initialize global vars.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-15-06:30pm] New.
 * @see    setup().
 */
void initVars() {
    Serial.print("Init global vars");

    // --- I/O. ---
    serialChar = '\0';
    memset(monitorCommand, '\0', sizeof(monitorCommand));
    memset(radioCommand,   '\0', sizeof(radioCommand));
    memset(rtcmSentence,   '\0', sizeof(rtcmSentence));

    // --- Operation. ---
    inLoop  = false;

    // --- Commands. ---
    testLEDr = false;
    testRad  = false;
    debugRad = false;
    reset    = false;

    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Initialize pins & pin values.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 * @see    setup().
 */
void initPins() {
    Serial.print("Init pins");
    pinMode(HC12_SET,  OUTPUT);                 // HC-12 - set pin for AT command mode.
    digitalWrite(HC12_SET,  HIGH);              // HC-12 - initially set pin for transparent mode.
    pinMode(LED_RADIO, OUTPUT);
    digitalWrite(LED_RADIO, LOW);
    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Start I2C Wire interfaces.
 * ------------------------------------------------
 *
 * Power only
 * 
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 * @see    setup().
 */
void startI2c() {

    // --- Start interfaces. ---
    // --- Register event functions. ---
}

/**
 * ------------------------------------------------
 *      Start tasks.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 * @see    Global vars: Task handles.
 * @see    setup().
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate.
 */
void startTasks() {

    // -- RTCM SEND status LED. --
    xTaskCreate(radioRtcmLEDtask,    "radio_RTCM_LED_task",       2048, NULL, 2, &radioRtcmLEDtaskHandle);
    vTaskSuspend(radioRtcmLEDtaskHandle);
    Serial.println("Task started: \"RTCM SEND status LED\".");
}

/**
 * ------------------------------------------------
 *      Start loop().
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 * @see    setup().
 */
void startLoop() {
    updateLED('0');     // RTCM LED off.
    inLoop = true;
    Serial.println("Loop() started.\n");
}   

/**
 * ============================================================================
 *                          Task functions.
 * ============================================================================
 * @see startTasks()              - Start tasks.
 *   @see radioRtcmLEDtask()      - Radio active LED.
 */

/**
 * ------------------------------------------------
 *      Task - Radio active LED.
 * ------------------------------------------------
 *
 * @param  void * pvParameters Pointer to task parameters.
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 * @see    startTasks().
 * @link   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 */
void radioRtcmLEDtask(void * pvParameters) {
    while(true) {
        digitalWrite(LED_RADIO, HIGH);      // LED on.
        vTaskDelay(LED_TIME_FLASH_ON);      // LED remains on (ms).
        digitalWrite(LED_RADIO, LOW);       // LED off.
        vTaskSuspend(NULL);                 // Suspend task.
    }
}

/**
 * ============================================================================
 *                          Loop functions.
 * ============================================================================
 */

/**
 * ------------------------------------------------
 *      Check serial USB for input.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-17-06:00pm] New.
 * @since  3.0.10 [2025-12-30-06:45pm] Refactor.
 * @see    loop().
 */
// void checkSerialUSBold() {  // ToDo: delete after testing new version.

//     // --- Local vars. ---
//     static char    monitorCommand[11];
//     static char    monitorBuffer[50];                               // Serial monitor buffer (C-string).
//     static uint8_t posn                = 0;                         // Input position for USB serial monitor command.
//     static uint8_t posnRad             = 0;                         // Input position for radio test command.
//            char    incomingByte;
//            uint8_t whichMonitorCommand = 0;                         // Which command was entered from the USB serial monitor.

//     // --- Read bytes. ---
//     if (Serial.available() > 0) {
//         incomingByte = Serial.read();

//         // -- Process input buffer. --
//         if ((incomingByte == '\n') || (incomingByte == '\r'))  {
//             memset(monitorCommand, '\0', sizeof(monitorCommand));
//             strncpy(monitorCommand, monitorBuffer, posn);
//             posn = 0;
//             memset(monitorBuffer, '\0', sizeof(monitorBuffer));
//             if (strstr(monitorCommand,"?") != NULL) {                                       // List commands.
//                 Serial.print("\nValid commands: ");
//                 for (size_t i = 0; i < NUM_COMMANDS-1; i++) {                               // Loop command array.
//                     if ((i != 0) && (i % 7 == 0)) {                                         // List a max of (7) commands per line.
//                         Serial.println();
//                     }
//                     Serial.printf("%s, ", COMMANDS[i]);
//                 }
//                 Serial.printf("%s.\n! to quit.\n", COMMANDS[NUM_COMMANDS-1]);
//             } else if (strstr(monitorCommand,"!") != NULL) {                                // Disable all debugs.
//                 testLEDr = false;
//                 testRad  = false;
//                 debugRad = false;
//                 reset    = false;
//                 Serial.println("\nAll debugging disabled.");
//             } else {
//                 whichMonitorCommand = 99;                                                   // Check which command.
//                 for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); i++) {       // Loop commands.
//                     if (strcmp(monitorCommand, COMMANDS[i]) == 0) {                         // Match a valid command.
//                         whichMonitorCommand = i;
//                         switch (whichMonitorCommand) {
//                             case 0:                                                         // Test the RTCM sentence relay LED.
//                                 testLEDr = (testLEDr == true) ? false : true;               // Flip the debug flag.
//                                 Serial.printf("%s %s\n", COMMANDS[whichMonitorCommand], (testLEDr  ? "enabled." : "disabled."));
//                                 break;
//                             case 1:                                                         // Test/config the radio.
//                                 testRad = (testRad == true) ? false : true;                 // Flip the debug flag.
//                                 Serial.printf("%s %s\n", COMMANDS[whichMonitorCommand], (testRad  ? "enabled." : "disabled."));
//                                 break;
//                             case 2:                                                         // Display data sent to radio.
//                                 debugRad = (debugRad == true) ? false : true;               // lip the debug flag.
//                                 Serial.printf("%s %s\n", COMMANDS[whichMonitorCommand], (debugRad ? "enabled." : "disabled."));
//                                 break;
//                             case 3:                                                         // Reset MCU.
//                                 reset = (reset == true) ? false : true;                     // Flip the debug flag.
//                                 Serial.printf("%s %s\n", COMMANDS[whichMonitorCommand], (reset       ? "enabled." : "disabled."));
//                                 Serial.println("Restarting ...");
//                                 esp_restart();                                              // Reset MCU.
//                         }

//                         // - Test the RTCM sentence relay LED. -
//                         if (testLEDr) {
//                             Serial.printf("Valid options: 0(off), 1(on), 2(active). %c to quit.\n", EXIT_TEST);
//                             while (true) {                                  // Infinite loop.
//                                 if (Serial.available() > 0) {
//                                     monitorChar = Serial.read();            // Read input from serial monitor.
//                                     Serial.read();                          // Discard newline.
//                                     switch (monitorChar) {
//                                         case EXIT_TEST:                     // All done.
//                                             Serial.println("testLEDr disabled.");
//                                             testLEDr = false;               // Clear test flag.
//                                             return;                         // Exit test mode.
//                                         case '0':                           // Radio LED - off.
//                                             Serial.printf("%c - radio LED off.\n", monitorChar);
//                                             updateLED('0');
//                                             break;
//                                         case '1':                           // BLE LED - on.
//                                             Serial.printf("%c - radio LED on.\n", monitorChar);
//                                             updateLED('1');
//                                             break;
//                                         case '2':                           // BLE LED - active.
//                                             Serial.printf("%c - radio LED active - 5 cycles.\n", monitorChar);
//                                             for (size_t i = 0; i < 5; i++) {
//                                                 updateLED('2');
//                                                 Serial.printf("Blink %i\n", i+1);
//                                                 delay(1000);
//                                             }
//                                             Serial.println();
//                                             break;
//                                         default:
//                                             Serial.printf("%c to quit. Valid options: 0(off), 1(on), 2(active).\n", EXIT_TEST);
//                                     }
//                                 }
//                             }
//                         }
//                         // -- End -- test relay LED. --

//                         // -- Test/config the radio. --
//                         if (testRad) {
//                             radioCommand[0] = '\0';                                         // Reset read buffer.
//                             posnRad = 0;
//                             digitalWrite(HC12_SET, LOW);
//                             Serial1.read();                                                 // Garbage first character.
//                             updateLED('0');                                                 // Radio LED off - AT command mode.
//                             Serial.println("\nHC-12 command mode enabled (! to exit)");     // Display instructions.
//                             Serial.println("  AT, AT+Bxxxx, AT+Cxxx, AT+FUx, AT+Px,");
//                             Serial.println("  AT+Ry (AT+RB, AT+RC, AT+RF, AT+RP, AT+RX),");
//                             Serial.println("  (y = B=baudrate, C=channel, F=mode, P=power),");
//                             Serial.println("  AT+Udps, AT+V, AT+SLEEP, AT+DEFAULT, AT+UPDATE.");
//                             Serial.println("  https://www.datsi.fi.upm.es/docencia/DMC/HC-12_v2.3A.pdf\n");
//                             while (true) {                                                  // Infinite loop.
//                                 if (Serial.available() > 0) {
//                                     monitorChar = Serial.read();                            // Read input from serial monitor.
//                                     if (monitorChar == EXIT_TEST) {                         // All done?
//                                         Serial.println("HC-12 command mode disabled.\n");
//                                         digitalWrite(HC12_SET, HIGH);
//                                         Serial.read();                                      // Clear the newline.
//                                         testRad = false;                                    // Clear test flag.
//                                         return;                                             // Exit test mode.
//                                     } else {
//                                         monitorChar = toupper(monitorChar);                 // Convert char to upper case.
//                                     }
//                                     switch (monitorChar) {
//                                         case '\n':                                          // Interact with HC-12.
//                                             Serial1.write(radioCommand);                    // Write command to HC-12.
//                                             Serial.println("");
//                                             delay(200);                                     // Allow HC-12 to process command & respond.
//                                             while (Serial1.available() > 0) {               // Read response from HC-12.
//                                                 monitorChar = '\0';
//                                                 monitorChar = Serial1.read();
//                                                 if ((255 != (int) monitorChar) && (posnRad > 0)) {  // Ignore first garbage character.
//                                                     Serial.print(monitorChar);              // Echo character to serial monitor.
//                                                 }
//                                             }
//                                             radioCommand[0] = '\0';                         // Reset read buffer.
//                                             posnRad=0;
//                                             break;
//                                         default:                                            // Echo & save input character.
//                                             if (255 != (int) monitorChar) {
//                                                 Serial.print(monitorChar);
//                                                 radioCommand[posnRad] = monitorChar;        // Add character to command buffer.
//                                                 posnRad++;
//                                             }
//                                     }
//                                 }
//                             }
//                         }
//                         // -- End -- Test/config the radio. --
//                     }                                                                       // End command match.
//                 }                                                                           // End loop commands.
//                 if ((whichMonitorCommand > 3) && (strlen(monitorCommand) > 0))  {
//                     Serial.printf("\n\"%s\" is not a valid command. \n", monitorCommand);   // Invalid command.
//                 }
//             }
//             memset(monitorCommand, '\0', sizeof(monitorCommand));                           // Reset buffer.  
//         } else {
//             monitorBuffer[posn] = incomingByte;                                             // Add to input buffer.
//             posn++;
//         }
//     }
// }

/**
 * ------------------------------------------------
 *      Check serial USB for input.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9  [2025-12-17-06:00pm] New.
 * @since  3.0.10 [2025-12-30-01:15pm] Refactor.
 * @see    loop().
 */
void checkSerialUSB() {

    // --- Local vars. ---
    static char   command[11];                                                  // Command buffer.
    static size_t posn         = 0;                                             // Input position for command buffer.
           size_t i            = 0;                                             // Command array elementcounter.
           size_t whichCommand = 0;                                             // Command array element matched.

    // --- Process input. ---
    if (Serial.available() > 0) {
        serialChar = Serial.read();                                             // Read char from USB Serial.
        if ((serialChar == '\n') || (serialChar == '\r'))  {                    // Possible command.
            if (posn > 0) {                                                     // Character(s) have been read from serial USB.
                if (strstr(command,"?") != NULL) {                              // List commands.
                    Serial.print("\nValid commands: ");
                    for (size_t i = 0; i < NUM_COMMANDS-1; i++) {               // Loop array of commands.
                        if ((i != 0) && (i % 7 == 0)) {                         // List max (7) commands per line.
                            Serial.println();
                        }
                        Serial.printf("%s, ", COMMANDS[i]);
                    }
                    Serial.printf("%s.\n! to quit.\n", COMMANDS[NUM_COMMANDS-1]);
                } else if (strstr(command,"!") != NULL) {                       // Disable all debugs.
                    testLEDr = false;
                    testRad  = false;
                    debugRad = false;
                    reset    = false;
                    Serial.println("\nAll debugging disabled.");
                } else {
                    size_t i            = 0;
                    size_t whichCommand = 100;                                                  // Which command was entered?
                    while ((whichCommand == 100) && (i < NUM_COMMANDS)) {
                        if (strcmp(command,COMMANDS[i]) == 0) {                                 // Match a valid command.
                            switch (i) {
                                case 0:                                                         // Test the RTCM sentence relay LED.
                                    testLEDr = (testLEDr == true) ? false : true;               // Flip the debug flag.
                                    Serial.printf("%s %s\n", COMMANDS[i], (testLEDr ? "enabled." : "disabled."));
                                    whichCommand = i;
                                    break;
                                case 1:                                                         // Test/config the radio.
                                    testRad = (testRad == true) ? false : true;                 // Flip the debug flag.
                                    Serial.printf("%s %s\n", COMMANDS[i], (testRad ? "enabled." : "disabled."));
                                    whichCommand = i;
                                    break;
                                case 2:                                                         // Display data from radio.
                                    debugRad = (debugRad == true) ? false : true;               // Flip the debug flag.
                                    Serial.printf("%s %s\n", COMMANDS[i], (debugRad ? "enabled." : "disabled."));
                                    whichCommand = i;
                                    break;
                                case 3:                                                         // Reset MCU.
                                    reset = (reset == true) ? false : true;                     // Flip the debug flag.
                                    Serial.printf("%s %s\n", COMMANDS[i], (reset ? "enabled." : "disabled."));
                                    Serial.println("Restarting ...");
                                    whichCommand = i;
                                    esp_restart();                                              // Reset MCU.
                            }
                            // -- Test/config the HC-12 radio. --
                            if (testRad) {
                                digitalWrite(HC12_SET, LOW);                        // Put HC-12 into command mode.
                                Serial1.write('\n');                                // Clear garbage.
                                delay(200);
                                while (Serial1.available() > 0) {
                                    Serial1.read();
                                }
                                Serial.println("\nHC-12 command mode enabled (! to exit)");     // Display instructions.
                                Serial.println("  AT, AT+Bxxxx, AT+Cxxx, AT+FUx, AT+Px,");
                                Serial.println("  AT+Ry (AT+RB, AT+RC, AT+RF, AT+RP, AT+RX),");
                                Serial.println("  (y = B=baudrate, C=channel, F=mode, P=power),");
                                Serial.println("  AT+Udps, AT+V, AT+SLEEP, AT+DEFAULT, AT+UPDATE.");
                                Serial.println("See https://www.datsi.fi.upm.es/docencia/DMC/HC-12_v2.3A.pdf\n");
                                memset(command, '\0', sizeof(command));                         // Reset read buffer.
                                posn = 0;
                                while (true) {                                                  // Infinite loop.
                                    if (Serial.available() > 0) {
                                        serialChar = toupper(Serial.read());                    // Read input from serial USB.
                                        if (serialChar == EXIT_TEST) {                          // All done?
                                            Serial.println("HC-12 command mode disabled.\n");
                                            digitalWrite(HC12_SET, HIGH);                       // Exit HC-12 command mode.
                                            Serial.read();                                      // Clear the newline.
                                            testRad = false;                                    // Clear test flag.
                                            return;                                             // Exit test mode.
                                        }
                                        switch (serialChar) {
                                            case '\n':                                          // Interact with HC-12.
                                                Serial1.write(command);                         // Write command to HC-12.
                                                Serial.println("");
                                                delay(200);                                     // Allow HC-12 to process command & respond.
                                                while (Serial1.available() > 0) {               // Read response from HC-12.
                                                    serialChar = Serial1.read();
                                                    if ((255 != (int) serialChar) && (posn > 0)) {  // Ignore first garbage character.
                                                        Serial.printf("%c", serialChar);            // Display character from HC-12.
                                                    }
                                                }
                                                memset(command, '\0', sizeof(command));         // Reset read buffer.
                                                posn=0;
                                                break;
                                            default:                                            // Echo & save input character.
                                                if (255 != (int) serialChar) {
                                                    Serial.print(serialChar);                   // Display character from serial USB.
                                                    command[posn] = serialChar;                 // Add character to command buffer.
                                                    posn++;
                                                }
                                        }
                                    }
                                }
                            }
                            // -- End -- Test/config the HC-12 radio. --

                            // -- Test radio LED. --
                            if (testLEDr) {
                                Serial.printf("Valid options: 0(off), 1(on), 2(active). %c to quit.\n", EXIT_TEST);
                                while (true) {                                  // Infinite loop.
                                    if (Serial.available() > 0) {
                                        serialChar = Serial.read();             // Read input from serial USB.
                                        Serial.read();                          // Discard newline.
                                        switch (serialChar) {
                                            case EXIT_TEST:                     // All done.
                                                Serial.println("testLEDr disabled.");
                                                testLEDr = false;               // Clear test flag.
                                                return;                         // Exit test mode.
                                            case '0':                           // Radio LED - off.
                                                Serial.printf("%c - radio LED off.\n", serialChar);
                                                updateLED('0');
                                                break;
                                            case '1':                           // BLE LED - on.
                                                Serial.printf("%c - radio LED on.\n", serialChar);
                                                updateLED('1');
                                                break;
                                            case '2':                           // BLE LED - active.
                                                Serial.printf("%c - radio LED active - 5 cycles.\n", serialChar);
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
                            // -- End -- Test radio LED. --
                        }                                                                       // End command match.
                        i++;
                    }                                                                           // End loop commands.
                    if ((strlen(command) > 0) && (whichCommand == 100))  {
                        Serial.printf("\n\"%s\" is not a valid command. \n", command);          // Invalid command.
                    }
                }
            }
            posn = 0;                                                                           // Reset for next command.
            memset(command, '\0', sizeof(command));
        } else {
            command[posn] = serialChar;                                                         // Add input to buffer.
            posn++;
        }
    }
}

/**
 * ------------------------------------------------
 *      Check serial 1 (EVK-RTCM). Send to serial 2 (HC-12 radio).
 * ------------------------------------------------
 * 
 * RTCM preamble = '11010011 000000xx' = 0xd3 0x00.
 *
 * @return void No output is returned.
 * @since  0.1.0  [2025-05-29-10:30pm] New.
 * @since  3.0.9  [2025-12-14-06:00pm] Version 3.
 * @since  3.0.10 [2025-12-14-06:00pm] Match Ghost_Rover.ino.
 * @see    Global vars: Serial.
 * @see    startSerialInterfaces().
 * @see    loop().
 * @link   https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3/blob/main/examples/ZED-F9P/Example3_StartRTCMBase/Example3_StartRTCMBase.ino.
 * @link   https://www.use-snip.com/kb/knowledge-base/an-rtcm-message-cheat-sheet/.
 * @link   https://www.use-snip.com/kb/knowledge-base/rtcm-3-message-list/.
 * @link   https://www.singularxyz.com/blog_detail/11.
 */
void checkRTCMtoRadio() {

    // -- Local vars. --
    static uint8_t  preamble  = 0;
    static uint16_t byteCount = 0;
           uint16_t msg_type  = 0;

    // -- Read Serial0 (EVK RTCM3) input. Send to Serial1 (HC-12 radio). --
    if (Serial0.available() > 0) {                                  // EVK RTCM3 data to read?
        serialChar = Serial0.read();                                // Read a character from Serial0 (EVK RTCM3) @ SERIAL0_SPEED.
        Serial1.write(serialChar);                                  // Write a character to Serial1 (HC-12 radio) @ SERIAL1_SPEED.
        if (serialChar == 0xd3) {                                   // Look for preamble (beginning of RTCM3 sentence).
            preamble = (preamble == 0) ? 1 : 2;                     // First (1) or new (2) preamble?
        }
        if (preamble == 1) {                                        // First preamble.
            rtcmSentence[byteCount] = serialChar;                   // Add byte to sentence buffer.
            byteCount++;                                            // Increment byte counter.
        } else if (preamble == 2) {                                 // New Preamble.
            if (debugRad) {                                         // Debug.
                msg_type = rtcm3GetMessageType(rtcmSentence);       // Parse message type.
                Serial.printf("\nRTCM3 %ld: %i bytes.\n",  msg_type, byteCount);
                for (size_t i = 0; i < byteCount; i++) {
                    Serial.printf("%02x ", rtcmSentence[i]);
                }
                Serial.println();
            }
            updateLED('2');                                         // Blink LED.
            byteCount = 0;
            preamble = 0;
            memset(rtcmSentence, '\0', sizeof(rtcmSentence));       // Clear the RTCM3 sentence buffer.
        }
    }
}

/**
 * Return RTCM3 message type.
 *
 * RTCM3 message structure:
 *   Byte 0: Preamble (0xD3).
 *   Byte 1-2: Reserved (6 bits) + Message length (10 bits).
 *   Byte 3-4: Message type (12 bits) + rest of message.
 *      - Message type starts at bit 24 (byte 3) and is 12 bits long.
 *      - It occupies the upper 8 bits of byte 3 and upper 4 bits of byte 4.
 *
 * @param  array RTCM3 sentence.
 * @return uint16_t Message type.
 * @since  0.8.7 [2025-12-16-06:00pm] New.
 * @see    checkRTCMtoRadio().
 * @link   https://portal.u-blox.com/s/question/0D52p0000C7MwDfCQK/can-you-find-out-the-message-type-of-a-given-rtcm3-message.
 */
uint16_t rtcm3GetMessageType(const char *buffer) {
    // Serial.printf("[%02x] [%02x] [%02x] [%02x] [%02x]\n", buffer[0],  buffer[1], buffer[2], buffer[3], buffer[3]);
    if (buffer[0] != 0xD3) {    // Check if preamble is correct
        return 0;               // Invalid preamble.
    }
    uint16_t message_type = ((uint16_t)buffer[3] << 4) | (buffer[4] >> 4);
    return message_type;
}

/**
 * ------------------------------------------------
 *      Toggle LEDs.
 * ------------------------------------------------
 *
 * @param  char ledR Radio LED.
 * @return void No output is returned.
 * @since  0.3.3 [2025-05-29-10:00pm] New.
 * @since  3.0.9 [2025-12-14-02:00pm] Version 3.
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

// --- Test. ---

/**
 * ============================================================================
 *                          Setup.
 * ============================================================================
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 */
void setup() {
    showBuild();                        // Show build & processor info.
    startSerial();                      // Start serial interfaces.
    initVars();                         // Initialize global vars.
    initPins();                         // Initialize pins & pin values.
    startTasks();                       // Start tasks.
    startLoop();                        // On to loop().
}

/**
 * ============================================================================
 *                          Task functions.
 * ============================================================================
 * @see startTasks()              - Start tasks.
 *   @see radioRtcmLEDtask()      - Radio active LED.
 */

/**
 * ============================================================================
 *                          Loop.
 * ============================================================================
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-14-02:00pm] New.
 */
void loop() {
    checkSerialUSB();                   // Check serial USB for input.
    checkRTCMtoRadio();                 // Check Serial0 for input (EVK RTCM), relay to Serial1 (HC-12 radio).
}
