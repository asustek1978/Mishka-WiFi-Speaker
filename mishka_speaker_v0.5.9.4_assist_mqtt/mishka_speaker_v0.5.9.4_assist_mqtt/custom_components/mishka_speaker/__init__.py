"""Mishka Speaker custom integration."""
from homeassistant.core import HomeAssistant
from homeassistant.config_entries import ConfigEntry
from .const import DOMAIN
from .diagnostics import MishkaDiagnostics

PLATFORMS = ["media_player", "light", "select", "sensor", "binary_sensor"]

async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    host = entry.options.get("host", entry.data.get("host", "192.168.1.111"))
    coordinator = MishkaDiagnostics(hass, host)
    # Diagnostics are optional; a temporarily offline HTTP endpoint must not
    # prevent MQTT radio and lighting entities from starting.
    await coordinator.async_refresh()
    hass.data.setdefault(DOMAIN, {})[entry.entry_id] = coordinator
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True

async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    result = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    if result:
        hass.data.get(DOMAIN, {}).pop(entry.entry_id, None)
    return result
