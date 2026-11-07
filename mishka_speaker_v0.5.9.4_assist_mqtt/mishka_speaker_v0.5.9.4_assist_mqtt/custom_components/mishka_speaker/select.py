"""Native favorite-station selector and LED effect selector."""
from __future__ import annotations
import json
from homeassistant.components import mqtt
from homeassistant.components.select import SelectEntity
from homeassistant.core import callback
from homeassistant.exceptions import HomeAssistantError
from .device import device_info

DISCOVERY_TOPIC = "homeassistant/select/mishka_wifi_station/config"
EFFECTS = ["Статичный", "Дыхание", "Бегущий огонь", "Светомузыка"]

async def async_setup_entry(hass, entry, async_add_entities):
    base = entry.data["base_topic"]
    async_add_entities([MishkaStation(base), MishkaEffect(base)])

class MishkaStation(SelectEntity):
    _attr_has_entity_name = True
    _attr_name = "Радиостанция"

    def __init__(self, base):
        self._base = base
        self._attr_unique_id = f"mishka_native_station_{base.replace('/', '_')}"
        self._attr_device_info = device_info(base)
        self._attr_options = []
        self._attr_current_option = None
        self._attr_available = False
        self._unsub = []

    async def async_added_to_hass(self):
        await super().async_added_to_hass()
        @callback
        def on_discovery(msg):
            try:
                data = json.loads(msg.payload)
                opts = data.get("options", [])
                if isinstance(opts, list) and all(isinstance(x, str) for x in opts):
                    self._attr_options = [x for x in opts if x.strip()]
                    self.async_write_ha_state()
            except (ValueError, TypeError, AttributeError):
                return
        @callback
        def on_station(msg):
            station = msg.payload.strip()
            self._attr_current_option = station if station in self._attr_options else None
            self.async_write_ha_state()
        @callback
        def on_availability(msg):
            self._attr_available = msg.payload.strip().lower() == "online"
            self.async_write_ha_state()
        self._unsub = [
            await mqtt.async_subscribe(self.hass, DISCOVERY_TOPIC, on_discovery, 1),
            await mqtt.async_subscribe(self.hass, f"{self._base}/station/state", on_station, 1),
            await mqtt.async_subscribe(self.hass, f"{self._base}/availability", on_availability, 1),
        ]

    async def async_select_option(self, option):
        if option not in self._attr_options:
            raise HomeAssistantError(f"Unknown station: {option}")
        await mqtt.async_publish(self.hass, f"{self._base}/station/set", option, qos=1)

    async def async_will_remove_from_hass(self):
        for unsub in self._unsub:
            unsub()
        self._unsub.clear()
        await super().async_will_remove_from_hass()

class MishkaEffect(SelectEntity):
    _attr_has_entity_name = True
    _attr_name = "Эффект подсветки"
    _attr_options = EFFECTS
    _attr_icon = "mdi:creation"

    def __init__(self, base):
        self._base = base
        self._attr_unique_id = f"mishka_native_effect_{base.replace('/', '_')}"
        self._attr_device_info = device_info(base)
        self._attr_current_option = None
        self._attr_available = False
        self._unsub = []

    async def async_added_to_hass(self):
        await super().async_added_to_hass()
        @callback
        def on_availability(msg):
            self._attr_available = msg.payload.strip().lower() == "online"
            self.async_write_ha_state()
        self._unsub = [await mqtt.async_subscribe(self.hass, f"{self._base}/availability", on_availability, 1)]

    async def async_select_option(self, option):
        if option not in EFFECTS:
            raise HomeAssistantError(f"Unknown effect: {option}")
        await mqtt.async_publish(self.hass, f"{self._base}/led/set", json.dumps({"effect": option}, ensure_ascii=False), qos=1)
        self._attr_current_option = option
        self.async_write_ha_state()

    async def async_will_remove_from_hass(self):
        for unsub in self._unsub:
            unsub()
        self._unsub.clear()
        await super().async_will_remove_from_hass()
