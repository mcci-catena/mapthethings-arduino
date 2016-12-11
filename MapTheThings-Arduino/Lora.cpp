#include "Catena4420.h"
#include "Arduino_LoRaWAN.h"
#include "Logging.h"
#include "Lora.h"

Catena4420 gCatena;
Catena4420::LoRaWAN gLoRaWAN;


static osjob_t timeoutjob;
static void txtimeout_func(osjob_t *job) {
  if (LMIC.opmode & OP_JOINING) {
     // keep waiting.. and don't time out.
     return;
  }
  digitalWrite(LED_BUILTIN, LOW); // off
  Log.Debug(F("Transmit Timeout" CR));
  //txActive = false;
  LMIC_clrTxData ();
}

static Arduino_LoRaWAN::SendBufferCbFn loraTxDone;

bool loraSendBytes(uint8_t *data, uint16_t len) {
  ostime_t t = os_getTime();
  //os_setTimedCallback(&txjob, t + ms2osticks(100), tx_func);
  // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Log.Debug(F("OP_TXRXPEND, not sending" CR));
        return false; // Did not enqueue
    } else {
        // Prepare upstream data transmission at the next possible time.
        Log.Debug(F("Packet queued" CR));
        digitalWrite(LED_BUILTIN, HIGH); // off
        if (! gLoRaWAN.SendBuffer(data, len, loraTxDone, data)){
            // failed immediately, don't time out.
            return false;
        }
        if (! (LMIC.opmode & OP_JOINING)) {
          // connection is up, message is queued:
          // Timeout TX after 20 seconds
          os_setTimedCallback(&timeoutjob, t + ms2osticks(20000), txtimeout_func);
        }
        return true;
    }
}

static void loraTxDone(void *pDoneCtx, bool fSuccess) {
  os_clearCallback(&timeoutjob);
  Log.Debug(F("%s: transmit complete %s" CR), __func__, fSuccess ? "OK" : "FAILED");
}

void setupLora() {
    gCatena.begin();
    CatenaSamd21::UniqueID_string_t CpuIDstring;

    gCatena.SafePrintf("CPU Unique ID: %s\n",
        gCatena.GetUniqueIDstring(&CpuIDstring)
        );

    /* find the platform */
    const CatenaSamd21::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

    const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

    if (pPlatform)
    {
      gCatena.SafePrintf("EUI64: ");
      for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
      {
        gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
      }
      gCatena.SafePrintf("\n");
      gCatena.SafePrintf(
            "Platform Flags:  %#010x\n",
            gCatena.GetPlatformFlags()
            );
      gCatena.SafePrintf(
            "Operating Flags:  %#010x\n",
            gCatena.GetOperatingFlags()
            );
    }

    gLoRaWAN.begin(&gCatena);

    // start the join process by sending a dummy message.
    static uint8_t d[1] = { 0 };
    loraSendBytes(d, sizeof(d));
}

void loopLora() {
    os_runloop_once();
}

void loraSetSF(uint sf) {
  dr_t dr;
  switch (sf) {
    case 7: dr = DR_SF7; break;
    case 8: dr = DR_SF8; break;
    case 9: dr = DR_SF9; break;
    case 10: dr = DR_SF10; break;
    default:
      dr = DR_SF10;
      Log.Debug(F("Invalid SF value: %d" CR), sf);
      break;
  }
  LMIC_setDrTxpow(dr,20);
}
