set(COMPONENT_ADD_INCLUDEDIRS inc)
set(COMPONENT_SRCDIRS src)
set(COMPONENT_PRIV_REQUIRES "drv_generic"
							"hal_global"
							"drv_led"
							"drv_relay"
							"drv_i2c"
							"drv_keyboard"
							"drv_pcf8574"
							"drv_lcd1602"
							"drv_rv8803")
register_component()

