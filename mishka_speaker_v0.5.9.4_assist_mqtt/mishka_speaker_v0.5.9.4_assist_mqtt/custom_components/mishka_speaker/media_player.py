"""Home Assistant media_player for ESP-IDF Mishka Speaker over MQTT."""
from __future__ import annotations
import json
import logging
from datetime import timedelta
from urllib.parse import quote, urlsplit, urlunsplit
from homeassistant.components import mqtt, media_source
from homeassistant.components.media_player import (
    MediaPlayerDeviceClass, MediaPlayerEntity, MediaPlayerEntityFeature, MediaPlayerState,
)
from homeassistant.core import callback
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity import EntityCategory
from .const import DOMAIN
from .device import device_info

_LOGGER = logging.getLogger(__name__)

async def async_setup_entry(hass, entry, async_add_entities):
    async_add_entities([MishkaSpeaker(entry.data["base_topic"], entry.entry_id)])

class MishkaSpeaker(MediaPlayerEntity):
    _attr_device_class = MediaPlayerDeviceClass.SPEAKER
    _attr_has_entity_name = True
    _attr_name = "Wi-Fi Speaker"
    _attr_icon = "mdi:speaker-wireless"
    _attr_supported_features = (
        MediaPlayerEntityFeature.PLAY
        | MediaPlayerEntityFeature.PAUSE
        | MediaPlayerEntityFeature.STOP
        | MediaPlayerEntityFeature.VOLUME_SET
        | MediaPlayerEntityFeature.VOLUME_STEP
        | MediaPlayerEntityFeature.PLAY_MEDIA
        | MediaPlayerEntityFeature.BROWSE_MEDIA
        | MediaPlayerEntityFeature.NEXT_TRACK
        | MediaPlayerEntityFeature.PREVIOUS_TRACK
        | MediaPlayerEntityFeature.SELECT_SOURCE
    )

    def __init__(self, base: str, entry_id: str):
        self._base = base
        self._attr_unique_id = f"mishka_speaker_{base.replace('/', '_')}"
        self._attr_device_info = device_info(base)
        self._attr_state = MediaPlayerState.IDLE
        self._attr_volume_level = 0.30
        self._attr_available = False
        self._attr_source_list = []
        self._attr_source = None
        self._attr_media_title = None
        self._unsub = []

    async def async_added_to_hass(self):
        await super().async_added_to_hass()

        @callback
        def state_message(msg):
            states = {
                "playing": MediaPlayerState.PLAYING,
                "paused": MediaPlayerState.PAUSED,
                "idle": MediaPlayerState.IDLE,
                "off": MediaPlayerState.OFF,
            }
            value = msg.payload.strip().lower()
            if value in states:
                self._attr_state = states[value]
                self.async_write_ha_state()
            else:
                _LOGGER.debug("Unexpected Mishka state: %s", value)

        @callback
        def volume_message(msg):
            try:
                volume = float(msg.payload)
            except (ValueError, TypeError):
                return
            if 0 <= volume <= 1:
                self._attr_volume_level = volume
                self.async_write_ha_state()

        @callback
        def availability_message(msg):
            self._attr_available = msg.payload.strip().lower() == "online"
            self.async_write_ha_state()

        @callback
        def station_message(msg):
            name = msg.payload.strip()
            self._attr_source = name or None
            self._attr_media_title = name or None
            self.async_write_ha_state()

        @callback
        def station_discovery(msg):
            try:
                payload = json.loads(msg.payload)
                options = payload.get("options", [])
                if not isinstance(options, list) or not all(
                    isinstance(item, str) for item in options
                ):
                    return
            except (ValueError, TypeError, AttributeError):
                _LOGGER.warning("Invalid Mishka station discovery payload")
                return
            self._attr_source_list = [name for name in options if name.strip()]
            self.async_write_ha_state()

        self._unsub = [
            await mqtt.async_subscribe(self.hass, f"{self._base}/state", state_message, 1),
            await mqtt.async_subscribe(self.hass, f"{self._base}/volume/state", volume_message, 1),
            await mqtt.async_subscribe(self.hass, f"{self._base}/availability", availability_message, 1),
            await mqtt.async_subscribe(self.hass, f"{self._base}/station/state", station_message, 1),
            await mqtt.async_subscribe(self.hass, "homeassistant/select/mishka_wifi_station/config", station_discovery, 1),
        ]

    async def async_will_remove_from_hass(self):
        for unsubscribe in self._unsub:
            unsubscribe()
        self._unsub.clear()
        await super().async_will_remove_from_hass()

    async def _command(self, command: str):
        await mqtt.async_publish(self.hass, f"{self._base}/command", command, qos=1)

    async def async_browse_media(self, media_content_type=None, media_content_id=None):
        """Expose the Home Assistant media library in the media browser."""
        return await media_source.async_browse_media(
            self.hass, media_content_id
        )

    @staticmethod
    def _encode_url_path(url: str) -> str:
        """Percent-encode non-ASCII characters without double-encoding %XX."""
        parts = urlsplit(url)
        return urlunsplit((parts.scheme, parts.netloc,
                           quote(parts.path, safe="/%:@!$&'()*+,;=-._~"),
                           parts.query, parts.fragment))

    async def async_play_media(self, media_type: str, media_id: str, **kwargs):
        """Resolve a HA media-source item to a playable URL and send it via MQTT."""
        if media_source.is_media_source_id(media_id):
            item = await media_source.async_resolve_media(
                self.hass, media_id, self.entity_id
            )
            url = item.url
            if url.startswith("/"):
                # Resolve relative HA media URLs against HA's LAN base URL.
                from homeassistant.helpers.network import get_url
                # Sign the media URL so the ESP32 can fetch it without
                # an Authorization header or a permanent HA access token.
                from homeassistant.components.http.auth import async_sign_path
                signed_path = async_sign_path(self.hass, url, timedelta(hours=2))
                url = get_url(self.hass, prefer_external=False).rstrip("/") + signed_path
        else:
            url = media_id
        url = self._encode_url_path(url)
        if not url.startswith(("http://", "https://")):
            raise HomeAssistantError("Mishka Speaker requires a playable HTTP(S) URL")
        # ESP32 must be able to reach this URL; authenticated HTTPS/redirects
        # and non-MP3 files may not be supported by the current firmware.
        await mqtt.async_publish(
            self.hass, f"{self._base}/media/url/set", url, qos=1
        )

    async def async_media_play(self):
        await self._command("PLAY")

    async def async_media_pause(self):
        await self._command("PAUSE")

    async def async_media_stop(self):
        await self._command("STOP")

    async def async_set_volume_level(self, volume):
        volume = max(0.0, min(1.0, volume))
        await mqtt.async_publish(self.hass, f"{self._base}/volume/set", f"{volume:.2f}", qos=1)

    async def async_volume_up(self):
        await self.async_set_volume_level((self.volume_level or 0) + 0.05)

    async def async_volume_down(self):
        await self.async_set_volume_level((self.volume_level or 0) - 0.05)

    async def async_media_next_track(self):
        await self._command("NEXT")

    async def async_media_previous_track(self):
        await self._command("PREVIOUS")

    async def async_select_source(self, source: str):
        if source not in self._attr_source_list:
            raise HomeAssistantError(f"Unknown Mishka station: {source}")
        await mqtt.async_publish(
            self.hass, f"{self._base}/station/set", source, qos=1
        )
