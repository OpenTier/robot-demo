# Battery event message carrying battery information.
string vehicle_id            # Identifier of the vehicle
bool is_charging             # Charging status
bool is_discharging          # Discharging status
uint32 time_to_fully_charge  # Time to complete charging (seconds)
uint32 estimated_range       # Estimated range (meters)
float32 battery_level        # Battery level (e.g., percentage or fraction)
float32 state_of_health      # Battery state of health (0.0 to 1.0)
float32 temperature          # Battery temperature (Celsius)