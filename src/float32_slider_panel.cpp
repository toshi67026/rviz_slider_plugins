#include "rviz_slider_plugins/float32_slider_panel.hpp"

#include <rviz_common/config.hpp>

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace rviz_slider_plugins {

Float32SliderPanel::Float32SliderPanel(QWidget *parent)
    : rviz_common::Panel(parent) {

  slider_ = new FloatSlider(Qt::Horizontal, this);
  connect(slider_, SIGNAL(floatValueChanged(float)), this,
          SLOT(valueChangeEvent(float)));

  QTimer *output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);

  // horizontal layout
  auto *hlayout1 = new QHBoxLayout;
  enable_check_ = new QCheckBox("enable");
  hlayout1->addWidget(enable_check_);
  hlayout1->addWidget(new QLabel("topic:"));
  topic_edit_ = new QLineEdit("float_value");
  hlayout1->addWidget(topic_edit_);

  auto *hlayout2 = new QHBoxLayout;
  min_edit_ = new QLineEdit("-1.0");
  max_edit_ = new QLineEdit("1.0");
  curr_slider_value_label_ = new QLabel(this);
  hlayout2->addWidget(new QLabel("min:"));
  hlayout2->addWidget(min_edit_);
  hlayout2->addWidget(curr_slider_value_label_);
  hlayout2->addWidget(new QLabel("max:"));
  hlayout2->addWidget(max_edit_);

  auto *hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(slider_);

  // vertical layout
  auto *layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addLayout(hlayout2);
  layout->addLayout(hlayout3);
  setLayout(layout);
}

void Float32SliderPanel::onInitialize() {
  nh_ =
      this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void Float32SliderPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

void Float32SliderPanel::valueChangeEvent(float value) {
  curr_slider_value_ = value;
}

void Float32SliderPanel::tick() {
  curr_slider_value_label_->setText(
      QString("curr: ") + QString::number(curr_slider_value_, 'f', 1));

  int min_value = min_edit_->text().toFloat() * 10.0;
  int max_value = max_edit_->text().toFloat() * 10.0;
  if (min_value <= max_value) {
    enable_check_->setEnabled(true);
    slider_->setRange(min_value, max_value);
    min_edit_->setStyleSheet("QLineEdit {background-color: white;}");
    max_edit_->setStyleSheet("QLineEdit {background-color: white;}");
  } else {
    enable_check_->setEnabled(false);
    min_edit_->setStyleSheet("QLineEdit {background-color: red;}");
    max_edit_->setStyleSheet("QLineEdit {background-color: red;}");
  }

  if (enable_check_->isChecked()) {
    topic_edit_->setEnabled(false);
    min_edit_->setEnabled(false);
    max_edit_->setEnabled(false);

    if (float32_publisher_) {
      auto msg = std::make_shared<std_msgs::msg::Float32>();
      msg->data = curr_slider_value_;
      float32_publisher_->publish(*msg);

    } else {
      std::string topic_name = topic_edit_->text().toStdString();
      if (topic_name != "") {
        float32_publisher_ = nh_->create_publisher<std_msgs::msg::Float32>(
            topic_name, rclcpp::QoS(10));
      }
    }
  } else {
    float32_publisher_.reset();
    topic_edit_->setEnabled(true);
    min_edit_->setEnabled(true);
    max_edit_->setEnabled(true);
  }
}

void Float32SliderPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
}

} // namespace rviz_slider_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_slider_plugins::Float32SliderPanel,
                       rviz_common::Panel)