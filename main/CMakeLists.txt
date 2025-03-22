set(component_srcs 
    "controller.cpp" 
    )

idf_component_register(
    SRCS "${component_srcs}"
    INCLUDE_DIRS "."
    REQUIRES driver
    REQUIRES esp_wifi
    REQUIRES esp_timer
    REQUIRES nvs_flash
    )
