#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using SetBool = std_srvs::srv::SetBool; rclcpp::Node::SharedPtr g_node = nullptr;

void my_handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<SetBool::Request> request,
                        const std::shared_ptr<SetBool::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(g_node->get_logger(),"My_callback has data %d",request->data);
    if (request->data==true)
    {
        /* code */
        response->success=false;
    }else{
        response->success=true;
    }
    response->message="return";
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("setbool_service_server");
    auto server = g_node->create_service<SetBool>("/my_service", my_handle_service);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}

