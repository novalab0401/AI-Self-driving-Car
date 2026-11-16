// Gazebo Harmonic model plugin: change a traffic light's lens colors at runtime.
//
// [GZ] Written new for Gazebo Harmonic. It keeps the interface of the Classic
// traffic_light_plugin in limo_ros2/limo_plugin (same topic, same color names,
// same link names), but none of its code: Classic repainted a lens by re-sending
// gazebo::msgs::Visual on the "~/visual" transport topic, which does not exist here.
//
// Harmonic can change a visual's material colors through the VisualCmd component
// (what the /world/<name>/material_color topic does internally). This plugin sets
// that component on the lens visuals, so the color changes in the GUI and in the
// camera sensor alike.
//
// Usage: send "red" | "yellow" | "green" | "off" on the Gazebo topic named in
// <topic> (default /traffic_light/color, gz.msgs.StringMsg). config/bridge.yaml
// connects it to the ROS topic of the same name (std_msgs/msg/String):
//
//   ros2 topic pub -1 /traffic_light/color std_msgs/msg/String "{data: 'green'}"
//
// The model (models/traffic_light/model.sdf) has one link per lens:
// red_light_link / yellow_light_link / green_light_link.

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Color.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/transport/Node.hh>

namespace limo_gz
{

class TrafficLight
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    this->model_ = gz::sim::Model(entity);
    if (!this->model_.Valid(ecm)) {
      gzerr << "traffic_light_plugin must be attached to a model." << std::endl;
      return;
    }

    // lens name -> (link name, full-brightness color when on)
    this->lenses_ = {
      {"red", {"red_light_link", gz::math::Color(1.0f, 0.0f, 0.0f, 1.0f)}},
      {"yellow", {"yellow_light_link", gz::math::Color(1.0f, 0.85f, 0.0f, 1.0f)}},
      {"green", {"green_light_link", gz::math::Color(0.0f, 1.0f, 0.0f, 1.0f)}},
    };

    const std::string topic = sdf->Get<std::string>("topic", "/traffic_light/color").first;
    this->state_ = sdf->Get<std::string>("initial_color", "red").first;

    if (!this->node_.Subscribe(topic, &TrafficLight::OnColor, this)) {
      gzerr << "traffic_light_plugin could not subscribe to [" << topic << "]" << std::endl;
      return;
    }
    this->configured_ = true;
    gzmsg << "traffic_light_plugin ready (topic: " << topic << ", initial color: "
          << this->state_ << ")" << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured_) {
      return;
    }

    std::string state;
    bool changed;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      state = this->state_;
      changed = this->changed_;
      this->changed_ = false;
    }

    // Apply on change, and again once a second. The renderer picks the command up
    // from its own thread and drops it if the lens visual does not exist there yet
    // (right after start), so re-applying lets the color settle by itself.
    const auto now = std::chrono::steady_clock::now();
    if (!changed && now - this->last_apply_ < std::chrono::seconds(1)) {
      return;
    }
    this->last_apply_ = now;

    for (const auto & [name, lens] : this->lenses_) {
      this->Paint(ecm, lens, name == state);
    }
  }

private:
  struct Lens
  {
    std::string link;
    gz::math::Color color;
  };

  /// Called from the transport thread when a color request arrives.
  void OnColor(const gz::msgs::StringMsg & msg)
  {
    const std::string & state = msg.data();
    if (this->lenses_.find(state) == this->lenses_.end() && state != "off") {
      gzwarn << "traffic_light_plugin: unknown color: " << state << std::endl;
      return;
    }
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = state;
    this->changed_ = true;
  }

  /// Turn one lens on (bright, glowing) or off (dark, no glow).
  void Paint(gz::sim::EntityComponentManager & ecm, const Lens & lens, bool on)
  {
    const gz::sim::Entity link = this->model_.LinkByName(ecm, lens.link);
    if (link == gz::sim::kNullEntity) {
      if (!this->warned_) {
        gzwarn << "traffic_light_plugin: link not found: " << lens.link << std::endl;
        this->warned_ = true;
      }
      return;
    }

    const gz::math::Color dim(
      lens.color.R() * 0.15f, lens.color.G() * 0.15f, lens.color.B() * 0.15f, 1.0f);
    const gz::math::Color black(0.0f, 0.0f, 0.0f, 1.0f);

    for (const gz::sim::Entity visual :
      ecm.ChildrenByComponents(link, gz::sim::components::Visual()))
    {
      gz::msgs::Visual msg;
      msg.set_id(visual);
      auto * material = msg.mutable_material();
      gz::msgs::Set(material->mutable_ambient(), on ? lens.color : dim);
      gz::msgs::Set(material->mutable_diffuse(), on ? lens.color : dim);
      gz::msgs::Set(material->mutable_specular(), black);
      gz::msgs::Set(material->mutable_emissive(), on ? lens.color : black);

      auto * cmd = ecm.Component<gz::sim::components::VisualCmd>(visual);
      if (cmd == nullptr) {
        ecm.CreateComponent(visual, gz::sim::components::VisualCmd(msg));
      } else {
        cmd->Data() = msg;
        ecm.SetChanged(
          visual, gz::sim::components::VisualCmd::typeId,
          gz::sim::ComponentState::OneTimeChange);
      }
    }
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::map<std::string, Lens> lenses_;
  gz::transport::Node node_;

  std::mutex mutex_;
  std::string state_{"red"};
  bool changed_{true};

  bool configured_{false};
  bool warned_{false};
  std::chrono::steady_clock::time_point last_apply_{};
};

}  // namespace limo_gz

GZ_ADD_PLUGIN(
  limo_gz::TrafficLight,
  gz::sim::System,
  limo_gz::TrafficLight::ISystemConfigure,
  limo_gz::TrafficLight::ISystemPreUpdate)
