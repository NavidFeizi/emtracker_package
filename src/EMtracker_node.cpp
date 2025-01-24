#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <blaze/Blaze.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/taskspace.hpp"

#include "EMTracker.hpp"
// #include "butterworth.hpp"

using namespace std::chrono_literals;

void print_position(double t, blaze::StaticVector<double, 3UL> tip, double sample_time);

class EMTrackerNode : public rclcpp::Node
{
protected:
public:
  EMTrackerNode()
      : Node("emtracker"), count_(0)
  {
    // Set default parameteres and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("sample_time", 16E-3);
    m_sample_time = this->get_parameter("sample_time").as_double();
    this->declare_parameter<double>("cutoff_freq", 6.6);
    m_cutoff_freq = this->get_parameter("cutoff_freq").as_double();

    m_filter = std::make_unique<ButterworthFilter<3UL>>(m_sample_time);
    m_filter->update_coeffs(m_cutoff_freq);

    std::string hostname = "/dev/ttyUSB0";
    EMTrackerNode::setup_emtracker(hostname);
    EMTrackerNode::setup_ros_interfaces();
    EMTrackerNode::setup_parameters_callback();
  }

  ~EMTrackerNode()
  {
    // Destructor will automatically close the hardware connection
  }

private:
  void setup_ros_interfaces()
  {
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_heartbeat = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_publisher = this->create_publisher<interfaces::msg::Taskspace>("/task_space/feedback", 10);
    // m_publisher_filtered = this->create_publisher<interfaces::msg::Taskspace>("EMTracker_org", 10);

    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_timer = this->create_wall_timer(
        sample_time, std::bind(&EMTrackerNode::read_callback, this), m_callback_group_read);
  }

  void setup_parameters_callback()
  {
    // Set up parameter callback
    auto param_callback =
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "cutoff_freq")
        {
          m_cutoff_freq = parameter.as_double();
          // m_emt->set_filter_params(0.5, m_cutoff_freq);
          m_filter->update_coeffs(m_cutoff_freq);
        }
      }
      // Update control logic if parameters are successfully updated
      if (result.successful)
      {
        RCLCPP_INFO(this->get_logger(), "Updated filter cutoff: %.2f Hz", m_cutoff_freq);
      }
      return result;
    };
    param_callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  void setup_emtracker(std::string hostname)
  {
    /** initialize emtracker **/
    double cutoff_freq = 30.0;                                                        //[Hz]
    m_emt = std::make_unique<EMTracker>(hostname, m_sample_time, cutoff_freq, false); // Allocate the object dynamically

    // // landmark registration process - uncomment only if you want to redo landmark registration
    // std::string landmarks = "landmarks_truth_base.csv";
    // std::string ref_sensor_name = "base"; // "base", "tool"
    // m_emt->landmark_registration(landmarks, ref_sensor_name);
    // EMTrackerNode::~EMTrackerNode();

    // start reading
    m_emt->start_read_thread();
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  }

  void read_callback()
  {
    auto msg = interfaces::msg::Taskspace();
    auto msg_org = interfaces::msg::Taskspace();
    // get current time
    rclcpp::Time now = this->get_clock()->now();

    double sample_time;
    quatTransformation tool_transform, tool_transform_dot;
    blaze::StaticVector<double, 3UL> tool_pos_flt, tool_vel_flt = blaze::StaticVector<double, 3UL>(0.0);

    m_emt->get_tool_transform_in_base(tool_transform);
    m_emt->get_tool_transform_in_base_dot(tool_transform_dot);
    m_emt->get_sample_time(sample_time);
    // emt->Get_Probe_Position(&pos_probe, &pos_probe_flt);

    tool_pos_flt = m_filter->add_data_point(tool_transform.translation);
    tool_vel_flt = (tool_pos_flt - m_tool_pos_flt_prev) / m_sample_time;
    m_tool_pos_flt_prev = tool_pos_flt;

    msg_org.p[0] = tool_transform.translation[1]; // to align with cathter robot system frame
    msg_org.p[1] = -1 * tool_transform.translation[0];
    msg_org.p[2] = tool_transform.translation[2];

    msg_org.q[0] = tool_transform_dot.translation[1];
    msg_org.q[1] = -1 * tool_transform_dot.translation[0];
    msg_org.q[2] = tool_transform_dot.translation[2];

    msg.p[0] = tool_pos_flt[1];
    msg.p[1] = -1 * tool_pos_flt[0];
    msg.p[2] = tool_pos_flt[2];

    msg.q[0] = tool_vel_flt[1];
    msg.q[1] = -1 * tool_vel_flt[0];
    msg.q[2] = tool_vel_flt[2];

    m_publisher->publish(msg_org);
    // m_publisher_filtered->publish(msg);

    double time = static_cast<double>(now.nanoseconds()) / 1E9;
    log_position(time, msg_org.p, sample_time);
    // log_position(time, Pos_tip_F, SampleTime);
  }

  void log_position(double t, blaze::StaticVector<double, 3> position, double sample_time)
  {
    std::ostringstream oss;
    auto print_with_space_if_positive = [](double value)
    {
      std::ostringstream tmp;
      tmp << std::fixed << std::setprecision(4);
      if (value >= 0)
      {
        tmp << " " << value;
      }
      else
      {
        tmp << value;
      }
      return tmp.str();
    };

    oss << "X:" << print_with_space_if_positive(position[0]) << "  "
        << "Y:" << print_with_space_if_positive(position[1]) << "  "
        << "Z:" << print_with_space_if_positive(position[2]) << " [m]"
        << "  |  dT:" << std::fixed << std::setprecision(1) << sample_time * 1e3 << " [ms]";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  size_t count_;
  double m_sample_time;
  double m_cutoff_freq;
  std::unique_ptr<EMTracker> m_emt;
  std::unique_ptr<ButterworthFilter<3UL>> m_filter;
  blaze::StaticVector<double, 3UL> m_tool_pos_flt_prev = blaze::StaticVector<double, 3UL>(0.0);

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_filtered;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_heartbeat_publisher;
  rclcpp::TimerBase::SharedPtr m_timer_heartbeat;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_heartbeat;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EMTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
