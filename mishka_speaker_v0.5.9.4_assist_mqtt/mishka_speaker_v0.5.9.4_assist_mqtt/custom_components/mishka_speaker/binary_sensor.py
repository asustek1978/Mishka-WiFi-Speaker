"""Mishka HTTP and device status diagnostics."""
from homeassistant.components.binary_sensor import BinarySensorEntity
from homeassistant.components import mqtt
from homeassistant.core import callback
from homeassistant.helpers.entity import EntityCategory
from homeassistant.helpers.update_coordinator import CoordinatorEntity
from .device import device_info

STATUS = (
    ("mqtt_connected", "MQTT подключён", "mdi:lan-connect"),
    ("microphone_muted", "Микрофон выключен", "mdi:microphone-off"),
    ("wake_inference_ready", "Распознавание готово", "mdi:microphone-message"),
)

async def async_setup_entry(hass, entry, async_add_entities):
    coordinator = hass.data["mishka_speaker"][entry.entry_id]
    base = entry.data["base_topic"]
    async_add_entities(
        [MishkaHTTPStatus(coordinator, base)]
        + [MishkaStatus(coordinator, base, *definition) for definition in STATUS]
        + [MishkaAssistActive(base)]
    )

class _MishkaDiagnostic(CoordinatorEntity, BinarySensorEntity):
    _attr_has_entity_name = True
    _attr_entity_category = EntityCategory.DIAGNOSTIC

    def __init__(self, coordinator, base):
        super().__init__(coordinator)
        self._attr_device_info = device_info(base)

class MishkaHTTPStatus(_MishkaDiagnostic):
    _attr_name = "HTTP диагностика доступна"
    _attr_icon = "mdi:web-check"

    def __init__(self, coordinator, base):
        super().__init__(coordinator, base)
        self._attr_unique_id = f"mishka_{base.replace('/', '_')}_diag_http_available"

    @property
    def is_on(self):
        return self.coordinator.http_available

class MishkaStatus(_MishkaDiagnostic):
    def __init__(self, coordinator, base, key, name, icon):
        super().__init__(coordinator, base)
        self._key = key
        self._attr_name = name
        self._attr_icon = icon
        self._attr_unique_id = f"mishka_{base.replace('/', '_')}_diag_{key}"

    @property
    def available(self):
        return self.coordinator.http_available and self._key in (self.coordinator.data or {})

    @property
    def is_on(self):
        return bool((self.coordinator.data or {}).get(self._key))


class MishkaAssistActive(BinarySensorEntity):
    """Assist session activity from firmware MQTT state, not HTTP polling.

    The firmware already publishes its own Assist state sensor via MQTT
    Discovery. This binary entity complements it without duplicating it.
    """

    _attr_has_entity_name = True
    _attr_entity_category = EntityCategory.DIAGNOSTIC
    _attr_name = "Голосовой ассистент активен"
    _attr_icon = "mdi:account-voice"

    def __init__(self, base):
        self._base = base
        self._attr_unique_id = f"mishka_{base.replace('/', '_')}_assist_active"
        self._attr_device_info = device_info(base)
        self._attr_available = False
        self._attr_is_on = False

    async def async_added_to_hass(self):
        await super().async_added_to_hass()

        @callback
        def on_assist_state(msg):
            value = msg.payload.strip().lower()
            # Values emitted by firmware v0.5.8.23.1.
            self._attr_is_on = value in {
                "authenticating", "pipeline_starting", "waiting_stt",
                "listening", "processing", "speaking",
            }
            self.async_write_ha_state()

        @callback
        def on_availability(msg):
            self._attr_available = msg.payload.strip().lower() == "online"
            if not self._attr_available:
                self._attr_is_on = False
            self.async_write_ha_state()

        unsub_state = await mqtt.async_subscribe(
            self.hass, f"{self._base}/assist/state", on_assist_state, 1
        )
        self.async_on_remove(unsub_state)
        unsub_availability = await mqtt.async_subscribe(
            self.hass, f"{self._base}/availability", on_availability, 1
        )
        self.async_on_remove(unsub_availability)
