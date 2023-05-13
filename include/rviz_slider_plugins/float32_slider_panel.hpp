#ifndef RVIZ_SLIDER_PLUGINS_FLOAT32_SLIDER_PANEL_HPP
#define RVIZ_SLIDER_PLUGINS_FLOAT32_SLIDER_PANEL_HPP

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif
#include <QSlider>

#include <std_msgs/msg/float32.hpp>

namespace rviz_slider_plugins {

class FloatSlider : public QSlider {
  Q_OBJECT

public:
  FloatSlider(Qt::Orientation orientation, QWidget *parent = 0)
      : QSlider(orientation, parent) {
    connect(this, SIGNAL(valueChanged(int)), this,
            SLOT(notifyValueChanged(int)));
  }

  void setFloatValue(float float_value) {
    this->setValue(static_cast<int>(float_value * 10.0));
  }

signals:
  void floatValueChanged(float value);

public slots:
  void notifyValueChanged(int value) {
    float float_value = value / 10.0;
    emit floatValueChanged(float_value);
  }
};

class Float32SliderPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  Float32SliderPanel(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void updateCurrValue(float value);
  void tick();

protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float32_publisher_;

  QCheckBox *enable_check_;
  QLineEdit *topic_edit_;
  QLineEdit *min_edit_;
  QLabel *curr_slider_value_label_;
  QLineEdit *max_edit_;
  FloatSlider *slider_;

private:
  float curr_slider_value_ = 0.0;
  void updateRange();
};

} // namespace rviz_slider_plugins

#endif // RVIZ_SLIDER_PLUGINS_FLOAT32_SLIDER_PANEL_HPP