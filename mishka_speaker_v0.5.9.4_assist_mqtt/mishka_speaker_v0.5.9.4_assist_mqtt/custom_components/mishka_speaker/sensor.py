"""Diagnostic sensors for Mishka Wi-Fi Speaker."""
from homeassistant.components.sensor import SensorEntity, SensorEntityDescription
from homeassistant.helpers.entity import EntityCategory
from homeassistant.helpers.update_coordinator import CoordinatorEntity
from .device import device_info

SENSORS = (
    SensorEntityDescription(key="version", name="Версия прошивки", icon="mdi:chip"),
    SensorEntityDescription(key="rssi", name="Wi-Fi RSSI", native_unit_of_measurement="dBm", icon="mdi:wifi"),
    SensorEntityDescription(key="heap", name="Свободная память", native_unit_of_measurement="B", icon="mdi:memory"),
    SensorEntityDescription(key="ssid", name="Wi-Fi SSID", icon="mdi:wifi-settings"),
    SensorEntityDescription(key="state", name="Состояние воспроизведения", icon="mdi:play-circle-outline"),
)

async def async_setup_entry(hass, entry, async_add_entities):
    coordinator = hass.data["mishka_speaker"][entry.entry_id]
    base = entry.data["base_topic"]
    async_add_entities([MishkaSensor(coordinator, base, description) for description in SENSORS])

class MishkaSensor(CoordinatorEntity, SensorEntity):
    _attr_entity_category = EntityCategory.DIAGNOSTIC
    _attr_has_entity_name = True

    def __init__(self, coordinator, base, description):
        super().__init__(coordinator)
        self.entity_description = description
        self._attr_unique_id = f"mishka_{base.replace('/', '_')}_diag_{description.key}"
        self._attr_device_info = device_info(base)

    @property
    def available(self):
        return self.coordinator.http_available and self.entity_description.key in (self.coordinator.data or {})

    @property
    def native_value(self):
        return (self.coordinator.data or {}).get(self.entity_description.key)
