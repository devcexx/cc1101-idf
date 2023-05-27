idf_component_register(SRCS "cc1101.c"
	            PRIV_REQUIRES driver freertos hal esp_timer
                    INCLUDE_DIRS ".")
