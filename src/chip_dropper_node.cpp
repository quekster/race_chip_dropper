#include <arpa/inet.h>    // NEW 2.6: htons, inet_pton
#include <sys/socket.h>   // NEW 2.6: socket, connect, send, recv, setsockopt
#include <sys/time.h>     // NEW 2.6: timeval
#include <unistd.h>       // NEW 2.6: close

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ChipDropper : public rclcpp::Node
{
public:
  ChipDropper() //constructor
  : Node("chip_dropper")
  {
    // Parameters first, so everything below can use them.
    send_to_dropper_ = declare_parameter<bool>("send_to_dropper", false);
    dropper_host_ = declare_parameter<std::string>("dropper_host", std::string("127.0.0.1"));
    dropper_port_ = declare_parameter<int>("dropper_port", 5555);   // NEW 2.6

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("chip_markers", 10);
    drop_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "chip_drop", 10,
      // SharedPtr form, not `const PointStamped &`: the robots run Foxy, whose rclcpp
      // only accepts pointer callbacks (const-reference ones arrived in Galactic).
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { on_drop(*msg); });

    RCLCPP_INFO(get_logger(), "chip_dropper is up, send_to_dropper=%s (dropper at %s:%d)",
      send_to_dropper_ ? "true" : "false", dropper_host_.c_str(), dropper_port_);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr drop_sub_;  // subscription to the chip_drop topic
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;   // publisher for the chip markers

  int next_id_ = 0; // unique ID for each marker

  bool send_to_dropper_;      // false = sim (print + marker only), true = really drop
  std::string dropper_host_;  // where the token-dropper service runs
  int dropper_port_;          // NEW 2.6: its TCP port

  void on_drop(const geometry_msgs::msg::PointStamped & msg) //-- runs for every chip_drop message
  {
    publish_marker(msg);  // publish a marker at the chip drop location

    // Sim: no dropper to talk to, just say so.
    if (!send_to_dropper_) {
      RCLCPP_INFO(get_logger(), "CHIP DROPPED (simulated) at x=%.2f, y=%.2f",
        msg.point.x, msg.point.y);
      return;
    }

    // Real dog: ask the token-dropper service to drop one chip. Blocks ~1.4 s.
    std::string reply = send_command("drop 1");
    if (reply == "OK") {
      RCLCPP_INFO(get_logger(), "CHIP DROPPED at x=%.2f, y=%.2f", msg.point.x, msg.point.y);
    } else {
      RCLCPP_ERROR(get_logger(), "chip drop FAILED at x=%.2f, y=%.2f: %s",
        msg.point.x, msg.point.y, reply.c_str());
    }
  }

  void publish_marker(const geometry_msgs::msg::PointStamped & msg) //-- publishes a marker at the chip drop location
  {
    visualization_msgs::msg::Marker m;
    m.header = msg.header;
    m.ns = "chips";
    m.id = next_id_++;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = msg.point;
    m.pose.orientation.w = 1.0; // no rotation
    m.scale.x = 0.4; // 40 cm wide...
    m.scale.y = 0.4;
    m.scale.z = 0.05; // ...5 cm tall: a flat disc
    m.color.r = 1.0; // orange
    m.color.g = 0.6;
    m.color.b = 0.0;
    m.color.a = 1.0; // fully opaque

    marker_pub_->publish(m);
  }

  // NEW 2.6: the TCP client. Sends one command line to the token-dropper service and
  // returns its reply line ("OK"), or an "ERR ..." of our own if anything went wrong.
  std::string send_command(const std::string & command)
  {
    // 1. Get a socket (the "phone"). AF_INET = IPv4, SOCK_STREAM = TCP.
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      return "ERR could not create socket";
    }

    // 2. Give up after 10 s instead of hanging forever if the service never answers.
    timeval timeout{};
    timeout.tv_sec = 10;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // 3. Write down the address to call. htons = put the port in network byte order.
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(dropper_port_);
    if (inet_pton(AF_INET, dropper_host_.c_str(), &addr.sin_addr) != 1) {
      close(sock);
      return "ERR bad dropper_host '" + dropper_host_ + "'";
    }

    // 4. Dial. Fails straight away when nothing is listening on that port.
    if (connect(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(sock);
      return "ERR cannot connect -- is the token-dropper service running?";
    }

    // 5. Say the command. The '\n' tells the service the command is complete.
    std::string line = command + "\n";
    if (send(sock, line.data(), line.size(), 0) < 0) {
      close(sock);
      return "ERR send failed";
    }

    // 6. Read the reply one character at a time until '\n'. recv returns 1 per
    //    character, 0 if the service hung up, -1 on the 10 s timeout -- either of
    //    the last two just ends the loop. '\r' is skipped (the bare ESP32 sends "\r\n").
    std::string reply;
    char c;
    while (recv(sock, &c, 1, 0) == 1 && c != '\n') {
      if (c != '\r') {
        reply += c;
      }
    }

    // 7. Hang up.
    close(sock);
    return reply.empty() ? "ERR no reply from the dropper (timed out)" : reply;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                       // start ROS
  rclcpp::spin(std::make_shared<ChipDropper>());  // make the node, run its callbacks until Ctrl+C
  rclcpp::shutdown();
  return 0;
}
