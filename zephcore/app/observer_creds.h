/*
 * SPDX-License-Identifier: Apache-2.0
 * Observer runtime credentials — stored in LittleFS, configured via serial CLI.
 *
 * All connection parameters are runtime-configurable so no credentials ever
 * appear in the codebase. File path: /lfs/repeater/obs_creds
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ObserverCreds {
	char wifi_ssid[64];      /* WiFi network name */
	char wifi_psk[64];       /* WiFi password (empty = open network) */
	char mqtt_host[128];     /* MQTT broker hostname, e.g. "abydos.hu" */
	uint16_t mqtt_port;      /* MQTT broker port, e.g. 8883 */
	uint8_t  mqtt_tls;       /* 1 = TLS (no cert verify), 0 = plaintext */
	char mqtt_user[64];      /* MQTT username */
	char mqtt_password[64];  /* MQTT password */
	char mqtt_iata[8];       /* IATA location code, e.g. "BUD", "BTS", "VIE" */
	uint8_t  _reserved[5];   /* alignment / future use */
};

/* Load creds from /lfs/repeater/obs_creds.
 * Returns true on success; on failure fills struct with safe zero defaults. */
bool observer_creds_load(struct ObserverCreds *creds, const char *base_path);

/* Save creds to /lfs/repeater/obs_creds. Returns true on success. */
bool observer_creds_save(const struct ObserverCreds *creds, const char *base_path);

/* Apply sensible defaults to a freshly-zeroed creds struct. */
static inline void observer_creds_init(struct ObserverCreds *creds)
{
	creds->mqtt_port = 8883;
	creds->mqtt_tls  = 1;
}

#ifdef __cplusplus
}
#endif
