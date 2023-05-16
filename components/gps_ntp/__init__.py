import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components.gps import gps_ns

DEPENDENCIES = ["gps"]
CODEOWNERS = ["@RobertJN64"]

GPS_NTP = gps_ns.class_("GPS_NTP_Server", cg.Component)

CONF_GPS_ID = "gps_ntp_id"
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GPS_NTP)
        }
    )
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # https://platformio.org/lib/show/1655/TinyGPSPlus
    cg.add_library("PaulStoffregen/Time", "1.6.1")