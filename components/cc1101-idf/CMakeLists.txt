idf_component_register(SRCS "cc1101.c"
			    "cc1101_priv.c"
	            PRIV_REQUIRES driver freertos hal esp_timer
                    INCLUDE_DIRS "include")
