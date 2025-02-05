idf_component_register(
    SRCS "src/WifiCom.cpp" "src/WifiConfig.cpp"
    INCLUDE_DIRS "src"
    REQUIRES esp_wifi nvs_flash
)
