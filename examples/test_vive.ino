/**
 * @file test_vive.ino
 * @brief Vive Positioning Sensor Test Program
 *
 * Tests both Vive sensors (TS3633-CM1) and displays:
 * - Connection status (0=No Signal, 1=Sync Only, 2=Receiving)
 * - X and Y coordinates (if receiving)
 * - Signal quality indicators
 *
 * Hardware:
 * - Vive Sensor 1: GPIO 34
 * - Vive Sensor 2: GPIO 35
 *
 * Status Codes:
 * - 0 = VIVE_NO_SIGNAL   (✗) - 没有检测到任何信号
 * - 1 = VIVE_SYNC_ONLY   (⚠) - 只检测到同步脉冲，缺少扫描信号
 * - 2 = VIVE_RECEIVING   (✓) - 正常接收完整信号
 */

#include <Arduino.h>

// Vive sensor pins
#define VIVE1_SIGNAL_GPIO 34
#define VIVE2_SIGNAL_GPIO 35

// Include Vive510 library
#include "src/include/vive510.h"

// Vive sensor objects
Vive510 vive1(VIVE1_SIGNAL_GPIO);
Vive510 vive2(VIVE2_SIGNAL_GPIO);

// Status names
const char* status_names[] = {
  "NO_SIGNAL",
  "SYNC_ONLY",
  "RECEIVING"
};

// Status symbols
const char* status_symbols[] = {
  "X",  // No signal
  "!",  // Sync only
  "OK"  // Receiving
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println();
  Serial.println("====================================================");
  Serial.println("     Vive Positioning Sensor Test Program");
  Serial.println("====================================================");
  Serial.println();

  // Initialize Vive sensors
  Serial.println("Initializing Vive sensors...");
  Serial.println();

  Serial.print("  Vive Sensor 1 (GPIO ");
  Serial.print(VIVE1_SIGNAL_GPIO);
  Serial.print(")... ");
  vive1.begin();
  Serial.println("Initialized");

  Serial.print("  Vive Sensor 2 (GPIO ");
  Serial.print(VIVE2_SIGNAL_GPIO);
  Serial.print(")... ");
  vive2.begin();
  Serial.println("Initialized");

  Serial.println();
  Serial.println("Waiting 2 seconds for sensors to stabilize...");
  delay(2000);

  Serial.println();
  Serial.println("Starting synchronization...");
  Serial.println();

  // Sync both sensors (5 repetitions = ~42ms per sensor)
  Serial.print("  Syncing Vive Sensor 1... ");
  int status1 = vive1.sync(5);
  Serial.print("Status: ");
  Serial.print(status1);
  Serial.print(" (");
  Serial.print(status_names[status1]);
  Serial.print(") ");
  Serial.println(status_symbols[status1]);

  Serial.print("  Syncing Vive Sensor 2... ");
  int status2 = vive2.sync(5);
  Serial.print("Status: ");
  Serial.print(status2);
  Serial.print(" (");
  Serial.print(status_names[status2]);
  Serial.print(") ");
  Serial.println(status_symbols[status2]);

  Serial.println();
  Serial.println("====================================================");
  Serial.println("Starting continuous monitoring...");
  Serial.println("====================================================");
  Serial.println();
  Serial.println("Format: [Status] X=#### Y=#### (if receiving)");
  Serial.println();
}

void loop() {
  // Read sensor 1
  int status1 = vive1.status();
  uint16_t x1 = vive1.xCoord();
  uint16_t y1 = vive1.yCoord();

  // Read sensor 2
  int status2 = vive2.status();
  uint16_t x2 = vive2.xCoord();
  uint16_t y2 = vive2.yCoord();

  // Display header
  Serial.println("----------------------------------------------------");

  // Display Sensor 1
  Serial.print("Sensor 1: [");
  Serial.print(status1);
  Serial.print("] ");
  Serial.print(status_symbols[status1]);
  Serial.print(" ");
  Serial.print(status_names[status1]);

  if (status1 == 2) {  // VIVE_RECEIVING
    Serial.print("  ->  X=");
    Serial.print(x1);
    Serial.print("  Y=");
    Serial.println(y1);
  } else {
    Serial.println();
    if (status1 == 0) {
      Serial.println("          -> Not connected or no signal");
    } else if (status1 == 1) {
      Serial.println("          -> Sync OK, missing sweep signals");
    }
  }



  // Display Sensor 2
  Serial.print("Sensor 2: [");
  Serial.print(status2);
  Serial.print("] ");
  Serial.print(status_symbols[status2]);
  Serial.print(" ");
  Serial.print(status_names[status2]);

  if (status2 == 2) {  // VIVE_RECEIVING
    Serial.print("  ->  X=");
    Serial.print(x2);
    Serial.print("  Y=");
    Serial.println(y2);
  } else {
    Serial.println();
    if (status2 == 0) {
      Serial.println("          -> Not connected or no signal");
    } else if (status2 == 1) {
      Serial.println("          -> Sync OK, missing sweep signals");
    }
  }

  Serial.println();

  // Summary
  Serial.println("Summary:");
  if (status1 == 2 && status2 == 2) {
    Serial.println("  >> Both sensors receiving! System working perfectly!");
  } else if (status1 == 2 || status2 == 2) {
    if (status1 == 2) {
      Serial.println("  >> Sensor 1 working!");
      Serial.println("  >> Sensor 2 not connected or not working");
    } else {
      Serial.println("  >> Sensor 1 not connected or not working");
      Serial.println("  >> Sensor 2 working!");
    }
  } else if (status1 == 1 || status2 == 1) {
    Serial.println("  >> Sync detected but no position data");
    Serial.println("     -> Lighthouse may be too far or blocked");
  } else {
    Serial.println("  >> No signals detected on either sensor");
    Serial.println("     -> Check power, wiring, and lighthouse");
  }

  Serial.println();

  // Try to re-sync if not receiving
  if (status1 != 2) {
    vive1.sync(5);
  }
  if (status2 != 2) {
    vive2.sync(5);
  }

  delay(1000);  // Update every 1 second
}
