#include <rclcpp/rclcpp.hpp>
#include "shuttlecock_detector_tree/servers/BackUpServer.hpp"
#include "shuttlecock_detector_tree/servers/DeliverShuttlecockServer.hpp"
#include "shuttlecock_detector_tree/servers/GetToShuttlecockServer.hpp"
#include "shuttlecock_detector_tree/servers/PickShuttlecockServer.hpp"
#include "shuttlecock_detector_tree/servers/RotateServer.hpp"
#include "shuttlecock_detector_tree/servers/SearchingShuttlecockServer.hpp"
#include "shuttlecock_detect/nodes/DistancesNode.hpp"


using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	auto distances = std::make_shared<Distances>();
	auto backUpServer = std::make_shared<BackUpServer>(options);
	auto deliverShuttlecockServer = std::make_shared<DeliverShuttlecockServer>(options);
	auto getToShuttlecockServer = std::make_shared<GetToShuttlecockServer>(options);
    auto pickShuttlecockServer = std::make_shared<PickShuttlecockServer>(options);
    auto rotateServer = std::make_shared<RotateServer>(options);
    auto searchingShuttlecockServer = std::make_shared<SearchingShuttlecockServer>(options);

	executor.add_node(distances);
	executor.add_node(backUpServer);
	executor.add_node(deliverShuttlecockServer);
	executor.add_node(getToShuttlecockServer);
    executor.add_node(pickShuttlecockServer);
    executor.add_node(rotateServer);
    executor.add_node(searchingShuttlecockServer);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}
