#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

class SubscriberAddressBook : public rclcpp::Node
{
public:
    SubscriberAddressBook()
        : Node("subscriber_address_book")
    {
        subscription_ = this->create_subscription<more_interfaces::msg::AddressBook>(
            "address_book",
            10,
            [this](const more_interfaces::msg::AddressBook::SharedPtr msg) -> void {
                std::cout
                    << "Subscriber Contact First:" << msg->first_name
                    << "  Last:" << msg->last_name
                    << std::endl;
            });
    }

private:
    rclcpp::Subscription<more_interfaces::msg::AddressBook>::SharedPtr subscription_;
};

int main(int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberAddressBook>());
    rclcpp::shutdown();
    return 0;
}