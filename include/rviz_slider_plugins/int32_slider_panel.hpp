#ifndef RVIZ_SLIDER_PLUGINS_INT32_SLIDER_PANEL_HPP
#define RVIZ_SLIDER_PLUGINS_INT32_SLIDER_PANEL_HPP

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <std_msgs/msg/int32.hpp>

namespace rviz_slider_plugins {

class Int32SliderPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  Int32SliderPanel(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void updateCurrValue(int value);
  void tick();

protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int32_publisher_;

  QCheckBox *enable_check_;
  QLineEdit *topic_edit_;
  QLineEdit *min_edit_;
  QLabel *curr_slider_value_label_;
  QLineEdit *max_edit_;
  QSlider *slider_;

private:
  int curr_slider_value_ = 0;
  void updateRange();
};

} // namespace rviz_slider_plugins

#endif // RVIZ_SLIDER_PLUGINS_INT32_SLIDER_PANEL_HPP