set(COMPONENT_ADD_INCLUDEDIRS inc)
set(COMPONENT_SRCDIRS src)
set(COMPONENT_PRIV_REQUIRES "drv_controller"
							"hal_global"
							"drv_i2c"
							"driver")
register_component()