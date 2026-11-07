"""UI configuration for Mishka Speaker."""
import voluptuous as vol
from homeassistant import config_entries
from homeassistant.helpers import selector
from .const import DOMAIN, DEFAULT_BASE_TOPIC

class MishkaSpeakerConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    VERSION = 1

    async def async_step_user(self, user_input=None):
        errors = {}
        if user_input is not None:
            topic = user_input["base_topic"].strip().strip("/")
            if not topic or any(ch in topic for ch in ("+", "#", " ")):
                errors["base"] = "invalid_topic"
            else:
                await self.async_set_unique_id(topic)
                self._abort_if_unique_id_configured()
                return self.async_create_entry(title="Mishka Wi-Fi Speaker", data={"base_topic": topic})
        return self.async_show_form(
            step_id="user",
            data_schema=vol.Schema({vol.Required("base_topic", default=DEFAULT_BASE_TOPIC): selector.TextSelector()}),
            errors=errors,
        )
