/*
 * SPDX-License-Identifier: Apache-2.0
 * ObserverMesh — listen-only LoRa mesh node implementation.
 */

#include "ObserverMesh.h"
#include "observer_creds.h"

#include <mesh/Utils.h>
#include <mesh/LoRaConfig.h>
#include <adapters/radio/LoRaRadioBase.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephcore_observer, CONFIG_ZEPHCORE_OBSERVER_LOG_LEVEL);

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

/* Forward declaration — implemented in ZephyrMQTTPublisher.c */
extern "C" {
	void mqtt_publisher_enqueue(const char *topic, const char *payload, int payload_len);
	bool mqtt_publisher_is_connected(void);
	void mqtt_publisher_reconnect(void);
}

/* Forward declaration — implemented in ZephyrWiFiStation.c */
extern "C" {
	bool zc_wifi_station_is_connected(void);
	void zc_wifi_station_reconnect(void);
	const char *zc_wifi_station_ssid(void);
}

namespace mesh {

/* ========== Construction ========== */

ObserverMesh::ObserverMesh(Radio &radio, MillisecondClock &ms, RNG &rng, RTCClock &rtc)
	: Dispatcher(radio, ms, _pkt_mgr),
	  _last_rssi(0.0f), _last_score(0.0f), _last_raw_len(0),
	  _store(nullptr), _creds(nullptr), _rng(&rng), _rtc(&rtc)
{
	memset(_pubkey_hex, 0, sizeof(_pubkey_hex));
	memset(_packets_topic, 0, sizeof(_packets_topic));
	memset(_status_topic, 0, sizeof(_status_topic));
}

/* ========== begin() ========== */

void ObserverMesh::begin(RepeaterDataStore *store, struct ObserverCreds *creds)
{
	_store = store;
	_creds = creds;

	/* Initialize prefs with observer-specific defaults */
	initNodePrefs(&_prefs);
	_prefs.cr           = 5;   /* CR 4/5 */
	_prefs.tx_power_dbm = 0;   /* observer never TXes anyway */
	/* freq=869.618, bw=62.5, sf=8 already set by initNodePrefs */

	/* Load persisted prefs (overrides defaults with saved values) */
	if (!_store->loadPrefs(_prefs)) {
		/* First boot — save observer defaults */
		_store->savePrefs(_prefs);
	}

	/* Load or generate node identity */
	if (!_store->loadIdentity(_self_id)) {
		LOG_INF("No identity found — generating new keypair");
		int attempts = 0;
		do {
			_self_id = LocalIdentity(_rng);
			attempts++;
		} while (attempts < 10 &&
			 (_self_id.pub_key[0] == 0x00 || _self_id.pub_key[0] == 0xFF));
		_store->saveIdentity(_self_id);
		LOG_INF("New observer identity saved");
	}

	/* Build hex pubkey string */
	Utils::toHex(_pubkey_hex, _self_id.pub_key, PUB_KEY_SIZE);
	_pubkey_hex[PUB_KEY_SIZE * 2] = '\0';

	/* Build MQTT topic strings */
	buildTopics();

	/* Log identity */
	LOG_INF("Observer ID: %.16s...", _pubkey_hex);

	/* Start radio in continuous RX mode */
	Dispatcher::begin();
}

void ObserverMesh::buildTopics()
{
	const char *iata = (_creds && _creds->mqtt_iata[0] != '\0')
			   ? _creds->mqtt_iata : "XXX";

	snprintf(_packets_topic, sizeof(_packets_topic),
		 "meshcore/%s/%s/packets", iata, _pubkey_hex);
	snprintf(_status_topic,  sizeof(_status_topic),
		 "meshcore/%s/%s/status",  iata, _pubkey_hex);
}

/* ========== RX hooks ========== */

void ObserverMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len)
{
	_last_rssi = rssi;
	_last_raw_len = (len <= (int)sizeof(_last_raw)) ? len : (int)sizeof(_last_raw);
	memcpy(_last_raw, raw, _last_raw_len);
}

void ObserverMesh::logRx(Packet *packet, int len, float score)
{
	(void)packet; (void)len;
	_last_score = score;
}

void ObserverMesh::enqueuePacket(Packet *pkt)
{
	/* Compute packet hash (8 bytes → 16 hex chars) */
	uint8_t hash_bytes[MAX_HASH_SIZE];
	char    hash_hex[MAX_HASH_SIZE * 2 + 1];
	pkt->calculatePacketHash(hash_bytes);
	Utils::toHex(hash_hex, hash_bytes, MAX_HASH_SIZE);
	hash_hex[MAX_HASH_SIZE * 2] = '\0';

	/* Encode raw wire bytes as hex */
	/* Each byte → 2 hex chars; max MAX_TRANS_UNIT=255 bytes → 510 chars + NUL */
	static char raw_hex[MAX_TRANS_UNIT * 2 + 1];
	Utils::toHex(raw_hex, _last_raw, _last_raw_len);
	raw_hex[_last_raw_len * 2] = '\0';

	/* Get current timestamp from RTC */
	uint32_t now_epoch = _rtc ? _rtc->getCurrentTime() : 0;
	struct tm tm_now;
	time_t t = (time_t)now_epoch;
	gmtime_r(&t, &tm_now);

	/* Format ISO 8601 timestamp (microseconds always 0 — RTC has 1s resolution) */
	char ts_buf[48];
	snprintf(ts_buf, sizeof(ts_buf),
		 "%04d-%02d-%02dT%02d:%02d:%02d.000000",
		 tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
		 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

	/* Format time and date fields matching meshcoretomqtt */
	char time_buf[12], date_buf[32];
	snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d",
		 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
	snprintf(date_buf, sizeof(date_buf), "%d/%d/%04d",
		 tm_now.tm_mday, tm_now.tm_mon + 1, tm_now.tm_year + 1900);

	/* Route letter: "F" = flood/transport-flood, "D" = direct/transport-direct */
	const char *route_str = pkt->isRouteDirect() ? "D" : "F";

	/* score in meshcoretomqtt format: integer score * 1000 */
	int score_int = (int)(_last_score * 1000.0f);

	/* Build JSON payload matching meshcoretomqtt packet format */
	static char json_buf[1024];
	int json_len = snprintf(json_buf, sizeof(json_buf),
		"{"
		"\"type\":\"PACKET\","
		"\"origin\":\"%s\","
		"\"origin_id\":\"%s\","
		"\"timestamp\":\"%s\","
		"\"direction\":\"rx\","
		"\"time\":\"%s\","
		"\"date\":\"%s\","
		"\"len\":\"%d\","
		"\"packet_type\":\"%u\","
		"\"route\":\"%s\","
		"\"payload_len\":\"%u\","
		"\"raw\":\"%s\","
		"\"SNR\":\"%d\","
		"\"RSSI\":\"%d\","
		"\"score\":\"%d\","
		"\"hash\":\"%s\""
		"}",
		_prefs.node_name,
		_pubkey_hex,
		ts_buf,
		time_buf,
		date_buf,
		_last_raw_len,
		(unsigned)pkt->getPayloadType(),
		route_str,
		(unsigned)pkt->payload_len,
		raw_hex,
		(int)pkt->getSNR(),
		(int)_last_rssi,
		score_int,
		hash_hex);

	if (json_len < 0 || json_len >= (int)sizeof(json_buf)) {
		LOG_WRN("Packet JSON truncated (len=%d)", json_len);
		json_len = (int)sizeof(json_buf) - 1;
	}

	mqtt_publisher_enqueue(_packets_topic, json_buf, json_len);
}

DispatcherAction ObserverMesh::onRecvPacket(Packet *pkt)
{
	/* Publish every reception — no deduplication.
	 * The same flood packet heard from different repeaters is published
	 * separately, each with its own SNR/RSSI (propagation data). */
	enqueuePacket(pkt);
	return ACTION_RELEASE;  /* never retransmit */
}

/* ========== Serial CLI ========== */

#define CLI_REPLY_SIZE 256

bool ObserverMesh::handleCLI(const char *command, char *reply, int reply_size)
{
	reply[0] = '\0';

	/* ---- help ---- */
	if (strcmp(command, "help") == 0 || command[0] == '\0') {
		return true;  /* caller prints the banner */
	}

	/* ---- get commands ---- */
	if (memcmp(command, "get ", 4) == 0) {
		const char *key = command + 4;

		if (strcmp(key, "role") == 0) {
			snprintf(reply, reply_size, "observer");

		} else if (strcmp(key, "name") == 0) {
			snprintf(reply, reply_size, "%s", _prefs.node_name);

		} else if (strcmp(key, "public.key") == 0) {
			snprintf(reply, reply_size, "%s", _pubkey_hex);

		} else if (strcmp(key, "board") == 0) {
#ifdef CONFIG_ZEPHCORE_BOARD_NAME
			snprintf(reply, reply_size, "%s", CONFIG_ZEPHCORE_BOARD_NAME);
#else
			snprintf(reply, reply_size, "unknown");
#endif
		} else if (strcmp(key, "version") == 0) {
			snprintf(reply, reply_size, "%s (%s)", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);

		} else if (strcmp(key, "radio") == 0) {
			snprintf(reply, reply_size,
				 "freq=%.3f bw=%.1f sf=%u cr=%u tx=%ddBm",
				 (double)_prefs.freq, (double)_prefs.bw,
				 _prefs.sf, _prefs.cr, _prefs.tx_power_dbm);

		} else if (strcmp(key, "wifi.ssid") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->wifi_ssid[0]) ? _creds->wifi_ssid : "(not set)");

		} else if (strcmp(key, "wifi.status") == 0) {
			snprintf(reply, reply_size, "%s",
				 zc_wifi_station_is_connected() ? "connected" : "disconnected");

		} else if (strcmp(key, "mqtt.status") == 0) {
			snprintf(reply, reply_size, "%s",
				 mqtt_publisher_is_connected() ? "connected" : "disconnected");

		} else if (strcmp(key, "mqtt.host") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->mqtt_host[0]) ? _creds->mqtt_host : "(not set)");

		} else if (strcmp(key, "mqtt.user") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->mqtt_user[0]) ? _creds->mqtt_user : "(not set)");

		} else if (strcmp(key, "mqtt.iata") == 0) {
			snprintf(reply, reply_size, "%s",
				 (_creds && _creds->mqtt_iata[0]) ? _creds->mqtt_iata : "(not set)");

		} else if (strcmp(key, "mqtt.port") == 0) {
			snprintf(reply, reply_size, "%u",
				 (_creds) ? (unsigned)_creds->mqtt_port : 8883u);

		} else if (strcmp(key, "mqtt.tls") == 0) {
			snprintf(reply, reply_size, "%u",
				 (_creds) ? (unsigned)_creds->mqtt_tls : 1u);

		} else {
			snprintf(reply, reply_size, "ERR unknown key: %s", key);
		}
		return false;
	}

	/* ---- set commands ---- */
	if (memcmp(command, "set ", 4) == 0) {
		const char *rest = command + 4;

		/* Helper: find value after "key " */
		auto find_val = [](const char *s, const char *prefix) -> const char * {
			size_t n = strlen(prefix);
			if (memcmp(s, prefix, n) == 0 && s[n] == ' ')
				return s + n + 1;
			return nullptr;
		};

		const char *val;

		if ((val = find_val(rest, "name")) != nullptr) {
			strncpy(_prefs.node_name, val, sizeof(_prefs.node_name) - 1);
			_prefs.node_name[sizeof(_prefs.node_name) - 1] = '\0';
			_store->savePrefs(_prefs);
			snprintf(reply, reply_size, "name=%s", _prefs.node_name);

		} else if ((val = find_val(rest, "freq")) != nullptr) {
			float f = (float)atof(val);
			/* Accept Hz (e.g. 869618000) or MHz (e.g. 869.618) */
			if (f > 1000000.0f) f /= 1000000.0f;
			if (f >= 150.0f && f <= 2500.0f) {
				_prefs.freq = f;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "freq=%.3f MHz", (double)_prefs.freq);
			} else {
				snprintf(reply, reply_size, "ERR freq out of range");
			}

		} else if ((val = find_val(rest, "sf")) != nullptr) {
			int sf = atoi(val);
			if (sf >= 7 && sf <= 12) {
				_prefs.sf = (uint8_t)sf;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "sf=%u", _prefs.sf);
			} else {
				snprintf(reply, reply_size, "ERR sf must be 7-12");
			}

		} else if ((val = find_val(rest, "bw")) != nullptr) {
			/* Accept index (0=125, 1=250, 2=500, 3=62.5, 4=41.7, 5=31.25)
			 * or kHz value directly */
			float bw;
			int idx = atoi(val);
			const float bw_table[] = { 125.0f, 250.0f, 500.0f, 62.5f, 41.7f, 31.25f };
			if (idx >= 0 && idx <= 5) {
				bw = bw_table[idx];
			} else {
				bw = (float)atof(val);
			}
			if (bw > 0.0f) {
				_prefs.bw = bw;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "bw=%.2f kHz", (double)_prefs.bw);
			} else {
				snprintf(reply, reply_size, "ERR invalid bw");
			}

		} else if ((val = find_val(rest, "cr")) != nullptr) {
			int cr = atoi(val);
			if (cr >= 5 && cr <= 8) {
				_prefs.cr = (uint8_t)cr;
				_store->savePrefs(_prefs);
				((LoRaRadioBase *)_radio)->reconfigureWithParams(
					_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
				snprintf(reply, reply_size, "cr=%u", _prefs.cr);
			} else {
				snprintf(reply, reply_size, "ERR cr must be 5-8");
			}

		} else if (!_creds) {
			snprintf(reply, reply_size, "ERR creds not initialized");

		} else if ((val = find_val(rest, "wifi.ssid")) != nullptr) {
			strncpy(_creds->wifi_ssid, val, sizeof(_creds->wifi_ssid) - 1);
			_creds->wifi_ssid[sizeof(_creds->wifi_ssid) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "wifi.ssid=%s (reconnecting)", _creds->wifi_ssid);
			zc_wifi_station_reconnect();

		} else if ((val = find_val(rest, "wifi.psk")) != nullptr) {
			strncpy(_creds->wifi_psk, val, sizeof(_creds->wifi_psk) - 1);
			_creds->wifi_psk[sizeof(_creds->wifi_psk) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "wifi.psk=*** (saved, reconnecting)");
			zc_wifi_station_reconnect();

		} else if ((val = find_val(rest, "mqtt.host")) != nullptr) {
			strncpy(_creds->mqtt_host, val, sizeof(_creds->mqtt_host) - 1);
			_creds->mqtt_host[sizeof(_creds->mqtt_host) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.host=%s (reconnecting)", _creds->mqtt_host);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.port")) != nullptr) {
			int port = atoi(val);
			if (port > 0 && port <= 65535) {
				_creds->mqtt_port = (uint16_t)port;
				observer_creds_save(_creds, _store->getBasePath());
				snprintf(reply, reply_size, "mqtt.port=%u (reconnecting)", _creds->mqtt_port);
				mqtt_publisher_reconnect();
			} else {
				snprintf(reply, reply_size, "ERR port must be 1-65535");
			}

		} else if ((val = find_val(rest, "mqtt.tls")) != nullptr) {
			int tls = atoi(val);
			_creds->mqtt_tls = (tls != 0) ? 1 : 0;
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.tls=%u (reconnecting)", _creds->mqtt_tls);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.user")) != nullptr) {
			strncpy(_creds->mqtt_user, val, sizeof(_creds->mqtt_user) - 1);
			_creds->mqtt_user[sizeof(_creds->mqtt_user) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.user=%s (reconnecting)", _creds->mqtt_user);
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.password")) != nullptr) {
			strncpy(_creds->mqtt_password, val, sizeof(_creds->mqtt_password) - 1);
			_creds->mqtt_password[sizeof(_creds->mqtt_password) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			snprintf(reply, reply_size, "mqtt.password=*** (saved, reconnecting)");
			mqtt_publisher_reconnect();

		} else if ((val = find_val(rest, "mqtt.iata")) != nullptr) {
			strncpy(_creds->mqtt_iata, val, sizeof(_creds->mqtt_iata) - 1);
			_creds->mqtt_iata[sizeof(_creds->mqtt_iata) - 1] = '\0';
			observer_creds_save(_creds, _store->getBasePath());
			buildTopics();  /* rebuild topic strings with new IATA */
			snprintf(reply, reply_size, "mqtt.iata=%s (topics updated, reconnecting)", _creds->mqtt_iata);
			mqtt_publisher_reconnect();

		} else {
			snprintf(reply, reply_size, "ERR unknown key");
		}
		return false;
	}

	snprintf(reply, reply_size, "ERR unknown command (type 'help')");
	return false;
}

} /* namespace mesh */
