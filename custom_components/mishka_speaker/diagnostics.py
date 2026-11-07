"""Best-effort HTTP diagnostics; MQTT control must remain independent."""
from __future__ import annotations

import asyncio
import logging
from datetime import timedelta
from urllib.parse import urlsplit

import aiohttp
from homeassistant.helpers.aiohttp_client import async_get_clientsession
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator

_LOGGER = logging.getLogger(__name__)


class MishkaDiagnostics(DataUpdateCoordinator):
    def __init__(self, hass, host: str):
        super().__init__(
            hass, _LOGGER, name="Mishka diagnostics",
            update_interval=timedelta(seconds=30),
        )
        # Accept a hostname/IP or a configured HTTP URL, but never allow a
        # malformed value to silently produce http://http://... URLs.
        raw = str(host).strip().rstrip("/")
        parsed = urlsplit(raw if "://" in raw else f"http://{raw}")
        if parsed.scheme not in ("http", "https") or not parsed.netloc:
            raise ValueError("Invalid Mishka diagnostics host")
        self.status_url = f"{parsed.scheme}://{parsed.netloc}/api/status"
        self.http_available = False
        self.last_error = None

    async def _async_update_data(self):
        try:
            timeout = aiohttp.ClientTimeout(total=5, connect=3)
            async with async_get_clientsession(self.hass).get(
                self.status_url, timeout=timeout
            ) as response:
                response.raise_for_status()
                data = await response.json(content_type=None)
                if not isinstance(data, dict):
                    raise ValueError("Unexpected status payload")
        except (aiohttp.ClientError, asyncio.TimeoutError, ValueError) as err:
            # Normal during reboot, OTA, or short Wi-Fi outages. Do not raise
            # UpdateFailed every 30 seconds; reflect it in entity availability.
            if self.http_available or self.last_error is None:
                _LOGGER.debug("Mishka HTTP diagnostics unavailable: %s", err)
            self.http_available = False
            self.last_error = type(err).__name__
            return {}
        if not self.http_available:
            _LOGGER.debug("Mishka HTTP diagnostics connected")
        self.http_available = True
        self.last_error = None
        return data
