"""Common MQTT-backed Mishka entity utilities."""
from homeassistant.helpers.device_registry import DeviceInfo
from .const import DOMAIN

def device_info(base: str) -> DeviceInfo:
    return DeviceInfo(
        identifiers={(DOMAIN, f"mishka_{base.replace('/', '_')}"), ("mqtt", "mishka_wifi_speaker")},
        name="МИШКА", manufacturer="DIY Waveshare", model="ESP32-S3 AUDIO",
    )
