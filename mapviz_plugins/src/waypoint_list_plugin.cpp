// *****************************************************************************
//
// Copyright (c) 2024, Southwest Research Institute (SwRI)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute (SwRI) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/waypoint_list_plugin.h>
#include <mapviz_plugins/topic_select.h>

// QT libraries
#include <QDialog>
#include <QOpenGLWidget>

// ROS libraries
#include <rclcpp/rclcpp.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <cstdio>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::WaypointListPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

WaypointListPlugin::WaypointListPlugin()
  : MapvizPlugin(),
    ui_(),
    config_widget_(new QWidget()),
    has_message_(false),
    topic_(""),
    qos_(rmw_qos_profile_default),
    current_seq_(0),
    source_frame_(""),
    stamp_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
  ui_.setupUi(config_widget_);
  ui_.waypoint_color->setColor(Qt::green);
  ui_.current_color->setColor(Qt::yellow);

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);

  // Set status text red
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p3);

  connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
  connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  connect(ui_.waypoint_color, SIGNAL(colorEdited(const QColor&)), this, SLOT(DrawIcon()));
  connect(ui_.current_color, SIGNAL(colorEdited(const QColor&)), this, SLOT(DrawIcon()));
}

void WaypointListPlugin::SelectTopic()
{
  auto [topic, qos] = SelectTopicDialog::selectTopic(
    node_,
    "mavros_msgs/msg/WaypointList",
    qos_);
  if (!topic.empty())
  {
    connectCallback(topic, qos);
  }
}

void WaypointListPlugin::TopicEdited()
{
  std::string topic = ui_.topic->text().trimmed().toStdString();
  connectCallback(topic, qos_);
}

void WaypointListPlugin::connectCallback(const std::string& topic, const rmw_qos_profile_t& qos)
{
  ui_.topic->setText(QString::fromStdString(topic));
  if ((topic != topic_) || !qosEqual(qos, qos_))
  {
    initialized_ = false;
    {
      std::lock_guard<std::mutex> lock(waypoints_mutex_);
      waypoints_.clear();
    }
    has_message_ = false;
    PrintWarning("No messages received.");

    waypoint_sub_.reset();

    topic_ = topic;
    qos_ = qos;
    if (!topic.empty())
    {
      waypoint_sub_ = node_->create_subscription<mavros_msgs::msg::WaypointList>(
        topic_,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
        std::bind(&WaypointListPlugin::waypointListCallback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_.c_str());
    }
  }
}

void WaypointListPlugin::waypointListCallback(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
  if (!has_message_)
  {
    initialized_ = true;
    has_message_ = true;
  }

  std::vector<TransformedWaypoint> new_waypoints;
  uint16_t new_current_seq = msg->current_seq;

  for (size_t i = 0; i < msg->waypoints.size(); i++)
  {
    const auto& wp = msg->waypoints[i];

    // Only process waypoints with valid global coordinates
    // Frame types 0, 3, 10, 11 are global frames (WGS84)
    if (wp.frame == mavros_msgs::msg::Waypoint::FRAME_GLOBAL ||
        wp.frame == mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT ||
        wp.frame == mavros_msgs::msg::Waypoint::FRAME_GLOBAL_TERRAIN_ALT ||
        wp.frame == mavros_msgs::msg::Waypoint::FRAME_GLOBAL_TERRAIN_ALT_INT)
    {
      TransformedWaypoint twp;
      // x_lat is latitude, y_long is longitude for global frames
      twp.point = tf2::Vector3(wp.x_lat, wp.y_long, wp.z_alt);
      twp.transformed = false;
      twp.is_current = wp.is_current || (i == new_current_seq);
      twp.command = wp.command;
      twp.seq = static_cast<uint16_t>(i);
      new_waypoints.push_back(twp);
    }
  }

  // Swap in the new data under lock
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    waypoints_ = std::move(new_waypoints);
    current_seq_ = new_current_seq;
    // Use WGS84 frame for global waypoints
    source_frame_ = "wgs84";
    if (node_) {
      stamp_ = node_->now();
    }
  }
}

void WaypointListPlugin::Transform()
{
  if (!initialized_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (waypoints_.empty() || source_frame_.empty())
  {
    return;
  }

  swri_transform_util::Transform transform;
  if (!GetTransform(source_frame_, stamp_, transform))
  {
    for (auto& wp : waypoints_)
    {
      wp.transformed = false;
    }
    PrintError("Failed to get transform from " + source_frame_);
    return;
  }

  for (auto& wp : waypoints_)
  {
    // For WGS84, point.x() is latitude, point.y() is longitude
    tf2::Vector3 geo_point(wp.point.y(), wp.point.x(), wp.point.z());
    tf2::Vector3 transformed = transform * geo_point;
    wp.transformed_point = transformed;
    wp.transformed = true;
  }
}

void WaypointListPlugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status, message);
}

void WaypointListPlugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status, message);
}

void WaypointListPlugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status, message);
}

QWidget* WaypointListPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);
  return config_widget_;
}

bool WaypointListPlugin::Initialize(QOpenGLWidget* canvas)
{
  canvas_ = canvas;
  DrawIcon();
  return true;
}

void WaypointListPlugin::DrawIcon()
{
  if (icon_)
  {
    QPixmap icon(16, 16);
    icon.fill(Qt::transparent);

    QPainter painter(&icon);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QPen pen(ui_.waypoint_color->color());
    pen.setWidth(2);
    painter.setPen(pen);

    // Draw a simple waypoint icon (circle with dot)
    painter.drawEllipse(3, 3, 10, 10);
    painter.setBrush(ui_.current_color->color());
    painter.drawEllipse(6, 6, 4, 4);

    icon_->SetPixmap(icon);
  }
}

void WaypointListPlugin::Draw(double x, double y, double scale)
{
  if (!initialized_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (waypoints_.empty())
  {
    PrintWarning("No waypoints to display.");
    return;
  }

  bool all_transformed = true;
  for (const auto& wp : waypoints_)
  {
    if (!wp.transformed)
    {
      all_transformed = false;
      break;
    }
  }

  if (!all_transformed)
  {
    return;
  }

  // Draw connections between waypoints first (so they appear behind)
  if (ui_.show_connections->isChecked())
  {
    DrawWaypointConnections();
  }

  // Draw each waypoint
  for (const auto& wp : waypoints_)
  {
    if (wp.transformed)
    {
      DrawWaypoint(wp, scale);
    }
  }

  // Draw highlight for current waypoint on top
  for (const auto& wp : waypoints_)
  {
    if (wp.transformed && wp.is_current)
    {
      DrawCurrentWaypointHighlight(wp, scale);
    }
  }

  PrintInfo("OK (" + std::to_string(waypoints_.size()) + " waypoints)");
}

void WaypointListPlugin::DrawWaypointConnections()
{
  const QColor color = ui_.waypoint_color->color();
  glColor4d(color.redF(), color.greenF(), color.blueF(), 0.5);
  glLineWidth(ui_.line_thickness->value());

  glBegin(GL_LINE_STRIP);
  for (const auto& wp : waypoints_)
  {
    if (wp.transformed)
    {
      glVertex2d(wp.transformed_point.x(), wp.transformed_point.y());
    }
  }
  glEnd();
}

void WaypointListPlugin::DrawWaypoint(const TransformedWaypoint& wp, double scale)
{
  const QColor color = ui_.waypoint_color->color();
  const double x = wp.transformed_point.x();
  const double y = wp.transformed_point.y();

  // Size in meters (adjust based on scale for visibility)
  double size = ui_.waypoint_size->value();

  // Draw filled circle for waypoint
  const int segments = 16;
  glColor4d(color.redF(), color.greenF(), color.blueF(), 0.7);
  glBegin(GL_TRIANGLE_FAN);
  glVertex2d(x, y);
  for (int i = 0; i <= segments; i++)
  {
    double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex2d(x + size * std::cos(angle), y + size * std::sin(angle));
  }
  glEnd();

  // Draw circle outline
  glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
  glLineWidth(ui_.line_thickness->value());
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < segments; i++)
  {
    double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex2d(x + size * std::cos(angle), y + size * std::sin(angle));
  }
  glEnd();
}

void WaypointListPlugin::DrawCurrentWaypointHighlight(const TransformedWaypoint& wp, double scale)
{
  const QColor color = ui_.current_color->color();
  const double x = wp.transformed_point.x();
  const double y = wp.transformed_point.y();

  // Larger highlight ring around current waypoint
  double size = ui_.waypoint_size->value() * 1.8;

  // Draw pulsing outer ring
  const int segments = 24;
  glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
  glLineWidth(ui_.line_thickness->value() * 2);
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < segments; i++)
  {
    double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex2d(x + size * std::cos(angle), y + size * std::sin(angle));
  }
  glEnd();

  // Draw inner filled circle
  double inner_size = ui_.waypoint_size->value() * 0.5;
  glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
  glBegin(GL_TRIANGLE_FAN);
  glVertex2d(x, y);
  for (int i = 0; i <= segments; i++)
  {
    double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex2d(x + inner_size * std::cos(angle), y + inner_size * std::sin(angle));
  }
  glEnd();
}

void WaypointListPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
  LoadQosConfig(node, qos_);
  if (node["topic"])
  {
    std::string topic = node["topic"].as<std::string>();
    ui_.topic->setText(topic.c_str());
    TopicEdited();
  }

  if (node["waypoint_color"])
  {
    std::string color = node["waypoint_color"].as<std::string>();
    QColor qcolor(color.c_str());
    ui_.waypoint_color->setColor(qcolor);
  }

  if (node["current_color"])
  {
    std::string color = node["current_color"].as<std::string>();
    QColor qcolor(color.c_str());
    ui_.current_color->setColor(qcolor);
  }

  if (node["waypoint_size"])
  {
    double size = node["waypoint_size"].as<double>();
    ui_.waypoint_size->setValue(size);
  }

  if (node["show_connections"])
  {
    bool show = node["show_connections"].as<bool>();
    ui_.show_connections->setChecked(show);
  }

  if (node["line_thickness"])
  {
    int thickness = node["line_thickness"].as<int>();
    ui_.line_thickness->setValue(thickness);
  }
}

void WaypointListPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
  std::string topic = ui_.topic->text().toStdString();
  emitter << YAML::Key << "topic" << YAML::Value << topic;

  std::string waypoint_color = ui_.waypoint_color->color().name().toStdString();
  emitter << YAML::Key << "waypoint_color" << YAML::Value << waypoint_color;

  std::string current_color = ui_.current_color->color().name().toStdString();
  emitter << YAML::Key << "current_color" << YAML::Value << current_color;

  emitter << YAML::Key << "waypoint_size" << YAML::Value << ui_.waypoint_size->value();
  emitter << YAML::Key << "show_connections" << YAML::Value << ui_.show_connections->isChecked();
  emitter << YAML::Key << "line_thickness" << YAML::Value << ui_.line_thickness->value();

  SaveQosConfig(emitter, qos_);
}

}  // namespace mapviz_plugins
