// =====================================================================================
//  Odyssey-10 Pro -- OpenDroneID broadcast transport implementation
// =====================================================================================

#include "odid_transport.h"
#include <Arduino.h>
#include <string.h>

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

// ASTM F3411 constants
#define ODID_BT_SERVICE_UUID_LO   0xFA
#define ODID_BT_SERVICE_UUID_HI   0xFF
#define ODID_BT_APP_CODE          0x0D
#define ODID_WIFI_OUI_0           0xFA
#define ODID_WIFI_OUI_1           0x0B
#define ODID_WIFI_OUI_2           0xBC
#define ODID_WIFI_OUI_TYPE        0x0D

#define ODID_MESSAGE_SIZE         25

static bool bleUp   = false;
static bool wifiUp  = false;
static uint8_t advBuffer[64];

// -------------------------------------------------------------------------------------
//  Bluetooth 5 Long Range extended advertising
// -------------------------------------------------------------------------------------
static esp_ble_gap_ext_adv_params_t extAdvParams = {
  .type            = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
  .interval_min    = 0x0140,          // 200 ms
  .interval_max    = 0x0140,
  .channel_map     = ADV_CHNL_ALL,
  .own_addr_type   = BLE_ADDR_TYPE_PUBLIC,
  .filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  .tx_power        = ESP_PWR_LVL_P9,
  // Coded PHY on both primary and secondary gives S=8 long range, which is what the
  // standard's "Bluetooth 5 Long Range" option means.
  .primary_phy     = ESP_BLE_GAP_PHY_CODED,
  .max_skip        = 0,
  .secondary_phy   = ESP_BLE_GAP_PHY_CODED,
  .sid             = 0,
  .scan_req_notif  = false,
};

static esp_ble_gap_ext_adv_t extAdvInstance[1] = {
  { .instance = 0, .duration = 0, .max_events = 0 },
};

static void bleGapCallback(esp_gap_ble_cb_event_t event,
                           esp_ble_gap_cb_param_t* /*param*/) {
  if (event == ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT) bleUp = true;
}

static void beginBle() {
  esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if (esp_bt_controller_init(&cfg) != ESP_OK) return;
  if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK) return;
  if (esp_bluedroid_init() != ESP_OK) return;
  if (esp_bluedroid_enable() != ESP_OK) return;

  esp_ble_gap_register_callback(bleGapCallback);
  esp_ble_gap_ext_adv_set_params(0, &extAdvParams);
  Serial.println("[ODID] BLE extended advertising configured (Coded PHY)");
}

// -------------------------------------------------------------------------------------
//  Wi-Fi beacon vendor IE
// -------------------------------------------------------------------------------------
static void beginWifi() {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&cfg) != ESP_OK) return;
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK) return;

  wifi_config_t apCfg = {};
  // A hidden SSID keeps the beacon out of everyone's network list while still
  // carrying the vendor IE that Remote ID receivers look for.
  strncpy((char*)apCfg.ap.ssid, "ODY-RID", sizeof(apCfg.ap.ssid) - 1);
  apCfg.ap.ssid_len       = strlen("ODY-RID");
  apCfg.ap.channel        = 6;
  apCfg.ap.authmode       = WIFI_AUTH_OPEN;
  apCfg.ap.ssid_hidden    = 1;
  apCfg.ap.max_connection = 0;
  apCfg.ap.beacon_interval = 1000;   // ms

  esp_wifi_set_config(WIFI_IF_AP, &apCfg);
  if (esp_wifi_start() != ESP_OK) return;
  wifiUp = true;
  Serial.println("[ODID] Wi-Fi beacon interface up on channel 6");
}

// -------------------------------------------------------------------------------------
void odidTransportBegin() {
  beginBle();
  beginWifi();
}

void odidTransportSend(uint8_t messageType, const uint8_t* encoded,
                       uint32_t length, uint8_t counter) {
  if (length != ODID_MESSAGE_SIZE) return;

  // ---- Bluetooth: Service Data AD structure ----------------------------------------
  //   [len][0x16][UUID lo][UUID hi][app code][counter][25-byte message]
  uint8_t i = 0;
  advBuffer[i++] = 3 + 1 + 1 + ODID_MESSAGE_SIZE;   // length byte, excludes itself
  advBuffer[i++] = 0x16;                            // Service Data, 16-bit UUID
  advBuffer[i++] = ODID_BT_SERVICE_UUID_LO;
  advBuffer[i++] = ODID_BT_SERVICE_UUID_HI;
  advBuffer[i++] = ODID_BT_APP_CODE;
  advBuffer[i++] = counter;
  memcpy(&advBuffer[i], encoded, ODID_MESSAGE_SIZE);
  i += ODID_MESSAGE_SIZE;

  esp_ble_gap_config_ext_adv_data_raw(0, i, advBuffer);
  esp_ble_gap_ext_adv_start(1, extAdvInstance);

  // ---- Wi-Fi: vendor-specific IE ----------------------------------------------------
  //   [0xDD][len][OUI 3 bytes][OUI type][counter][25-byte message]
  if (wifiUp) {
    static vendor_ie_data_t* ie = nullptr;
    static uint8_t ieStorage[6 + 1 + ODID_MESSAGE_SIZE + 8];
    ie = (vendor_ie_data_t*)ieStorage;
    ie->element_id   = WIFI_VENDOR_IE_ELEMENT_ID;      // 0xDD
    ie->length       = 3 + 1 + 1 + ODID_MESSAGE_SIZE;
    ie->vendor_oui[0] = ODID_WIFI_OUI_0;
    ie->vendor_oui[1] = ODID_WIFI_OUI_1;
    ie->vendor_oui[2] = ODID_WIFI_OUI_2;
    ie->vendor_oui_type = ODID_WIFI_OUI_TYPE;
    ie->payload[0] = counter;
    memcpy(&ie->payload[1], encoded, ODID_MESSAGE_SIZE);

    esp_wifi_set_vendor_ie(true, WIFI_VND_IE_TYPE_BEACON, WIFI_VND_IE_ID_0, ie);
  }

  (void)messageType;
}

bool odidTransportHealthy() {
  return bleUp && wifiUp;
}
