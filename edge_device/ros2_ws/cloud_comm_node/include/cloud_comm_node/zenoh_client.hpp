#pragma once

#include <string>
#include <functional>
#include <unordered_map>
#include <memory>

#include "zenoh.hxx"

namespace cloud_comm_node
{

    class ZenohClient
    {
    public:
        ZenohClient() : m_session(zenoh::Session::open(zenoh::Config::create_default()))
        {
            // Initialize publishers
            m_robot_status_publisher = std::make_unique<zenoh::Publisher>(
                m_session.declare_publisher("cloud/telemetry/robot_status"));
            m_battery_status_publisher = std::make_unique<zenoh::Publisher>(
                m_session.declare_publisher("cloud/telemetry/edge_device/battery"));
        }

        // Publish data to a specific resource
        void publish_robot_status(const std::string &data)
        {
            m_robot_status_publisher->put(data);
        }

        void publish_edge_device_battery(const std::string &data)
        {
            m_battery_status_publisher->put(data);
        }

        // Subscribe to robot commands for a given robot ID
        void subscribe_to_robot_command(const std::string &robot_id,
                                        std::function<void(const std::string &)> callback)
        {
            std::string topic = "vehicle/" + robot_id + "/robot_control";
            m_robot_command_subscribers[robot_id] = std::make_unique<zenoh::Subscriber<void>>(
                m_session.declare_subscriber(
                    topic,
                    // on_sample callback
                    [callback](const zenoh::Sample &sample)
                    {
                        callback(sample.get_payload().as_string());
                    },
                    []() {}));
        }

        // Optionally, add an unsubscribe method if needed:
        void unsubscribe_robot_command(const std::string &robot_id)
        {
            m_robot_command_subscribers.erase(robot_id);
        }

    private:
        zenoh::Session m_session;
        std::unique_ptr<zenoh::Publisher> m_robot_status_publisher;
        std::unique_ptr<zenoh::Publisher> m_battery_status_publisher;

        // Map to hold subscribers for each robot by its ID
        std::unordered_map<std::string, std::unique_ptr<zenoh::Subscriber<void>>> m_robot_command_subscribers;
    };

} // namespace cloud_comm_node
