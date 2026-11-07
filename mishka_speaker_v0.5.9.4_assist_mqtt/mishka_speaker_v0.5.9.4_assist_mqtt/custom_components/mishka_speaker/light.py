"""Native light entity for Mishka WS2812, using existing MQTT topics."""
from __future__ import annotations
import json
from homeassistant.components import mqtt
from homeassistant.components.light import LightEntity, ColorMode, ATTR_BRIGHTNESS, ATTR_RGB_COLOR
from homeassistant.core import callback
from .device import device_info

async def async_setup_entry(hass, entry, async_add_entities):
    async_add_entities([MishkaLight(entry.data["base_topic"])])

class MishkaLight(LightEntity):
    _attr_has_entity_name = True
    _attr_name = "Подсветка"
    _attr_supported_color_modes = {ColorMode.RGB}
    _attr_color_mode = ColorMode.RGB

    def __init__(self, base):
        self._base = base
        self._attr_unique_id = f"mishka_native_light_{base.replace('/', '_')}"
        self._attr_device_info = device_info(base)
        self._attr_is_on = False
        self._attr_available = False
        self._attr_brightness = 128
        self._attr_rgb_color = (255, 255, 255)
        self._unsub = []

    async def async_added_to_hass(self):
        await super().async_added_to_hass()
        @callback
        def on_state(msg):
            try:
                data = json.loads(msg.payload)
                self._attr_is_on = data.get("state") == "ON"
                self._attr_brightness = max(0, min(255, int(data.get("brightness", 128))))
                c = data.get("color", {})
                if isinstance(c, dict):
                    self._attr_rgb_color = tuple(max(0, min(255, int(c.get(k, 255)))) for k in ("r", "g", "b"))
                self.async_write_ha_state()
            except (ValueError, TypeError, AttributeError):
                return
        @callback
        def on_availability(msg):
            self._attr_available = msg.payload.strip().lower() == "online"
            self.async_write_ha_state()
        self._unsub = [
            await mqtt.async_subscribe(self.hass, f"{self._base}/led/state", on_state, 1),
            await mqtt.async_subscribe(self.hass, f"{self._base}/availability", on_availability, 1),
        ]

    async def async_will_remove_from_hass(self):
        for unsub in self._unsub:
            unsub()
        self._unsub.clear()
        await super().async_will_remove_from_hass()

    async def async_turn_on(self, **kwargs):
        data = {"state": "ON"}
        if ATTR_BRIGHTNESS in kwargs:
            data["brightness"] = kwargs[ATTR_BRIGHTNESS]
        if ATTR_RGB_COLOR in kwargs:
            r, g, b = kwargs[ATTR_RGB_COLOR]
            data["color"] = {"r": r, "g": g, "b": b}
        await mqtt.async_publish(self.hass, f"{self._base}/led/set", json.dumps(data), qos=1)

    async def async_turn_off(self, **kwargs):
        await mqtt.async_publish(self.hass, f"{self._base}/led/set", '{"state":"OFF"}', qos=1)
