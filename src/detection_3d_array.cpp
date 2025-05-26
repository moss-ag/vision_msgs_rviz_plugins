// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vision_msgs_rviz_plugins/detection_3d_array.hpp>

#include <memory>

namespace rviz_plugins
{

Detection3DArrayDisplay::Detection3DArrayDisplay()
{
  only_edge_property_ = new rviz_common::properties::BoolProperty(
    "Only Edge", false, "Display only edges of the boxes", this, SLOT(updateEdge()));
  line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 0.05, "Line width of edges", this, SLOT(updateLineWidth()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Transparency", this, SLOT(updateAlpha()));
  show_score_property_ = new rviz_common::properties::BoolProperty(
    "Show Score", false, "Display score next to bounding boxes", this, SLOT(updateShowScores()));
  string_property_ = new rviz_common::properties::StringProperty(
    "ConfigPath", "", "Path to yaml config for rgb color mappings", this,
    SLOT(updateColorConfigs()));
  confidence_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Confidence Threshold", 0.5, "Minimum confidence for detections to be displayed.", this, SLOT(updateThreshold()));
    autocompute_colors_property_ = new rviz_common::properties::BoolProperty(
      "Autocompute Colors", false, "Automatically compute colors for each detection class", this,
      SLOT(updateAutocomputeColors()));
}

Detection3DArrayDisplay::~Detection3DArrayDisplay()
{
  delete only_edge_property_;
  delete line_width_property_;
  delete alpha_property_;
  delete show_score_property_;
}

void Detection3DArrayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  m_marker_common->initialize(context_, scene_node_);

  topic_property_->setValue("detection3_d_array");
  topic_property_->setDescription("Detection3DArray topic to subscribe to.");

  line_width_property_->setMax(0.1);
  line_width_property_->setMin(0.01);
  line_width_property_->hide();

  alpha_property_->setMax(1.0);
  alpha_property_->setMin(0.1);

  confidence_threshold_property_->setMax(1.0);
  confidence_threshold_property_->setMin(0.0);

  line_width = line_width_property_->getFloat();
  alpha = alpha_property_->getFloat();

  only_edge_ = only_edge_property_->getBool();
  show_score_ = show_score_property_->getBool();
  confidence_threshold_ = confidence_threshold_property_->getFloat();
}

void Detection3DArrayDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  m_marker_common->load(config);
}

void Detection3DArrayDisplay::processMessage(
  vision_msgs::msg::Detection3DArray::ConstSharedPtr msg)
{
  latest_msg = msg;

  auto filtered_msg = filterMessage(latest_msg);
  if (!only_edge_) {
    showBoxes(filtered_msg, show_score_);
  } else {
    showEdges(filtered_msg, show_score_);
  }
}

vision_msgs::msg::Detection3DArray::ConstSharedPtr
Detection3DArrayDisplay::filterMessage(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) const
{
  auto filtered_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
  filtered_msg->header = msg->header;
  for (const auto& detection: msg->detections) {
    if (detection.results.empty()) { continue; }

    if (detection.results[0].hypothesis.score >= confidence_threshold_) {
      filtered_msg->detections.push_back(detection);
    }
  }

  return filtered_msg;
}

void Detection3DArrayDisplay::update(float wall_dt, float ros_dt)
{
  m_marker_common->update(wall_dt, ros_dt);
}

void Detection3DArrayDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common->clearMarkers();
  edges_.clear();
}

void Detection3DArrayDisplay::updateAutocomputeColors()
{
  if (autocompute_colors_property_->getBool()) {
    this->updateColorConfig();
  }

  // Update the color immediately
  updateEdge();
}

void Detection3DArrayDisplay::updateEdge()
{
  only_edge_ = only_edge_property_->getBool();
  if (only_edge_) {
    line_width_property_->show();
  } else {
    line_width_property_->hide();
  }
  // Imediately apply attribute
  if (latest_msg) {
    auto filtered_msg = filterMessage(latest_msg);
    if (only_edge_) {
      showEdges(filtered_msg, show_score_);
    } else {
      showBoxes(filtered_msg, show_score_);
    }
  }
}

void Detection3DArrayDisplay::updateLineWidth()
{
  line_width = line_width_property_->getFloat();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateAlpha()
{
  alpha = alpha_property_->getFloat();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateShowScores()
{
  show_score_ = show_score_property_->getBool();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void Detection3DArrayDisplay::updateColorConfigs()
{
  this->updateColorConfig();
}

void Detection3DArrayDisplay::updateThreshold() {
  confidence_threshold_ = confidence_threshold_property_->getFloat();
  if (latest_msg) {
    reset();
    processMessage(latest_msg);
  }
}

}  // namespace rviz_plugins

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Detection3DArrayDisplay, rviz_common::Display)
