idf_component_register(
    SRCS "src/Robot.cpp"
    INCLUDE_DIRS "src"
    REQUIRES SabertoothRC SolenoidKicker MecanumDrive BatterySensor LedTelltale
)
