idf_component_register(
    SRCS "src/HttpController.cpp" "src/RosController.cpp"
    INCLUDE_DIRS "src"
    REQUIRES esp_http_server Robot micro_ros_espidf_component
)
