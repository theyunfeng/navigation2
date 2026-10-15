// Copyright (c) 2020 Samsung Research Russia
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
// limitations under the License. Reserved.

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"

using namespace std::chrono_literals;

static const char FILTER_NAME[]{"keepout_filter"};
static const char INFO_TOPIC[]{"costmap_filter_info"};
static const char MASK_TOPIC[]{"mask"};

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher(double base, double multiplier)
  : Node("costmap_filter_info_pub")
  {
    publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      INFO_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<nav2_msgs::msg::CostmapFilterInfo> msg =
      std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = 0;
    msg->filter_mask_topic = MASK_TOPIC;
    msg->base = static_cast<float>(base);
    msg->multiplier = static_cast<float>(multiplier);

    publisher_->publish(std::move(msg));
  }

  ~InfoPublisher()
  {
    publisher_.reset();
  }

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};  // InfoPublisher

class MaskPublisher : public rclcpp::Node
{
public:
  explicit MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
  : Node("mask_pub")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      MASK_TOPIC,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    publisher_->publish(mask);
  }

  ~MaskPublisher()
  {
    publisher_.reset();
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};  // MaskPublisher

struct Point
{
  unsigned int x, y;
};

class TestNode : public ::testing::Test
{
public:
  TestNode() {}

  ~TestNode() {}

protected:
  void createMaps(unsigned char master_value, int8_t mask_value, const std::string & mask_frame);
  void createFarMask(int8_t mask_value, double origin_x, double origin_y);
  void publishMaps();
  void rePublishInfo(double base, double multiplier);
  void rePublishMask();
  void waitSome(const std::chrono::nanoseconds & duration);
  void createKeepoutFilter(const std::string & global_frame);
  void createTFBroadcaster(const std::string & mask_frame, const std::string & global_frame);
  void verifyMasterGrid(unsigned char free_value, unsigned char keepout_value);
  void testStandardScenario(unsigned char free_value, unsigned char keepout_value);
  void testFramesScenario(unsigned char free_value, unsigned char keepout_value);
  void testFarMaskScenario();
  void reset();

  std::shared_ptr<nav2_costmap_2d::KeepoutFilter> keepout_filter_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master_grid_;

  std::vector<Point> keepout_points_;

private:
  nav2_util::LifecycleNode::SharedPtr node_;

  // 过滤器保存 LayeredCostmap 裸指针，需与过滤器同生命周期
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<geometry_msgs::msg::TransformStamped> transform_;

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> mask_;

  std::shared_ptr<InfoPublisher> info_publisher_;
  std::shared_ptr<MaskPublisher> mask_publisher_;
};

void TestNode::createMaps(
  unsigned char master_value, int8_t mask_value, const std::string & mask_frame)
{
  // Make map and mask put as follows:
  //
  //  map             (10,10)
  //   *----------------*
  //   |   mask (6,6)   |
  //   |    *-----*     |
  //   |    |/////|     |
  //   |    |/////|     |
  //   |    *-----*     |
  //   |  (3,3)         |
  //   *----------------*
  // (0,0)

  const double resolution = 1.0;

  // Create master_grid_
  unsigned int width = 10;
  unsigned int height = 10;
  master_grid_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
    width, height, resolution, 0.0, 0.0, master_value);

  // Create mask_
  width = 3;
  height = 3;
  mask_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  mask_->info.resolution = resolution;
  mask_->header.frame_id = mask_frame;
  mask_->info.width = width;
  mask_->info.height = height;
  mask_->info.origin.position.x = 3.0;
  mask_->info.origin.position.y = 3.0;
  mask_->info.origin.position.z = 0.0;
  mask_->info.origin.orientation.x = 0.0;
  mask_->info.origin.orientation.y = 0.0;
  mask_->info.origin.orientation.z = 0.0;
  mask_->info.origin.orientation.w = 1.0;
  mask_->data.resize(width * height, mask_value);
}

void TestNode::createFarMask(int8_t mask_value, double origin_x, double origin_y)
{
  // Small 3x3 mask placed at (origin_x, origin_y), e.g. far from the master costmap origin:
  //
  //  map (50x50)              (50,50)
  //   *------------------------*
  //   |            mask (43,43)|
  //   |              *---*     |
  //   |              |///|     |
  //   |              *---*     |
  //   |           (40,40)      |
  //   *------------------------*
  // (0,0)
  const double resolution = 1.0;
  const unsigned int width = 3;
  const unsigned int height = 3;
  mask_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  mask_->info.resolution = resolution;
  mask_->header.frame_id = "map";
  mask_->info.width = width;
  mask_->info.height = height;
  mask_->info.origin.position.x = origin_x;
  mask_->info.origin.position.y = origin_y;
  mask_->info.origin.orientation.w = 1.0;
  mask_->data.resize(width * height, mask_value);
}

void TestNode::publishMaps()
{
  info_publisher_ = std::make_shared<InfoPublisher>(0.0, 1.0);
  mask_publisher_ = std::make_shared<MaskPublisher>(*mask_);
}

void TestNode::rePublishInfo(double base, double multiplier)
{
  info_publisher_.reset();
  info_publisher_ = std::make_shared<InfoPublisher>(base, multiplier);
  // Allow both CostmapFilterInfo and filter mask subscribers
  // to receive a new message
  waitSome(100ms);
}

void TestNode::rePublishMask()
{
  mask_publisher_.reset();
  mask_publisher_ = std::make_shared<MaskPublisher>(*mask_);
  // Allow filter mask subscriber to receive a new message
  waitSome(100ms);
}

void TestNode::waitSome(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= rclcpp::Duration(duration)) {
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
}

void TestNode::createKeepoutFilter(const std::string & global_frame)
{
  node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(global_frame, false, false);

  node_->declare_parameter(
    std::string(FILTER_NAME) + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".transform_tolerance", 0.5));
  node_->declare_parameter(
    std::string(FILTER_NAME) + ".filter_info_topic", rclcpp::ParameterValue(INFO_TOPIC));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".filter_info_topic", INFO_TOPIC));

  keepout_filter_ = std::make_shared<nav2_costmap_2d::KeepoutFilter>();
  keepout_filter_->initialize(
    layers_.get(), std::string(FILTER_NAME),
    tf_buffer_.get(), node_, nullptr);
  keepout_filter_->initializeFilter(INFO_TOPIC);

  // Wait until mask will be received by KeepoutFilter
  while (!keepout_filter_->isActive()) {
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
}

void TestNode::createTFBroadcaster(const std::string & mask_frame, const std::string & global_frame)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
  transform_->header.frame_id = mask_frame;
  transform_->child_frame_id = global_frame;

  transform_->header.stamp = node_->now();
  transform_->transform.translation.x = 1.0;
  transform_->transform.translation.y = 1.0;
  transform_->transform.translation.z = 0.0;
  transform_->transform.rotation.x = 0.0;
  transform_->transform.rotation.y = 0.0;
  transform_->transform.rotation.z = 0.0;
  transform_->transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(*transform_);

  // Allow tf_buffer_ to be filled by listener
  waitSome(100ms);
}

void TestNode::verifyMasterGrid(unsigned char free_value, unsigned char keepout_value)
{
  unsigned int x, y;
  bool is_checked;

  for (y = 0; y < master_grid_->getSizeInCellsY(); y++) {
    for (x = 0; x < master_grid_->getSizeInCellsX(); x++) {
      is_checked = false;
      for (std::vector<Point>::iterator it = keepout_points_.begin();
        it != keepout_points_.end(); it++)
      {
        if (x == it->x && y == it->y) {
          EXPECT_EQ(master_grid_->getCost(x, y), keepout_value);
          is_checked = true;
          break;
        }
      }
      if (!is_checked) {
        EXPECT_EQ(master_grid_->getCost(x, y), free_value);
      }
    }
  }
}

void TestNode::testStandardScenario(unsigned char free_value, unsigned char keepout_value)
{
  geometry_msgs::msg::Pose2D pose;
  // Intersection window: added 4 points
  keepout_filter_->process(*master_grid_, 2, 2, 5, 5, pose);
  keepout_points_.push_back(Point{3, 3});
  keepout_points_.push_back(Point{3, 4});
  keepout_points_.push_back(Point{4, 3});
  keepout_points_.push_back(Point{4, 4});
  verifyMasterGrid(free_value, keepout_value);
  // Two windows outside on the horisontal/vertical edge: no new points added
  keepout_filter_->process(*master_grid_, 3, 6, 5, 7, pose);
  keepout_filter_->process(*master_grid_, 6, 3, 7, 5, pose);
  verifyMasterGrid(free_value, keepout_value);
  // Corner window: added 1 point
  keepout_filter_->process(*master_grid_, 5, 5, 6, 6, pose);
  keepout_points_.push_back(Point{5, 5});
  verifyMasterGrid(free_value, keepout_value);
  // Outside windows: no new points added
  keepout_filter_->process(*master_grid_, 0, 0, 2, 2, pose);
  keepout_filter_->process(*master_grid_, 0, 7, 2, 9, pose);
  keepout_filter_->process(*master_grid_, 7, 0, 9, 2, pose);
  keepout_filter_->process(*master_grid_, 7, 7, 9, 9, pose);
  verifyMasterGrid(free_value, keepout_value);
}

void TestNode::testFramesScenario(unsigned char free_value, unsigned char keepout_value)
{
  geometry_msgs::msg::Pose2D pose;
  // Intersection window: added all 9 points because of map->odom frame shift
  keepout_filter_->process(*master_grid_, 2, 2, 5, 5, pose);
  keepout_points_.push_back(Point{2, 2});
  keepout_points_.push_back(Point{2, 3});
  keepout_points_.push_back(Point{2, 4});
  keepout_points_.push_back(Point{3, 2});
  keepout_points_.push_back(Point{3, 3});
  keepout_points_.push_back(Point{3, 4});
  keepout_points_.push_back(Point{4, 2});
  keepout_points_.push_back(Point{4, 3});
  keepout_points_.push_back(Point{4, 4});
  verifyMasterGrid(free_value, keepout_value);
}

void TestNode::testFarMaskScenario()
{
  // 使用真实 LayeredCostmap::updateMap()：窗口完全由 KeepoutFilter::updateBounds() 决定
  layers_->resizeMap(50, 50, 1.0, 0.0, 0.0);
  layers_->addFilter(keepout_filter_);
  nav2_costmap_2d::Costmap2D * costmap = layers_->getCostmap();

  // Expect lethal cells only inside the 3x3 cells starting at (lethal_x, lethal_y);
  // negative coordinates mean the whole costmap should be free
  auto verify = [costmap](int lethal_x, int lethal_y) {
      for (int y = 0; y < static_cast<int>(costmap->getSizeInCellsY()); y++) {
        for (int x = 0; x < static_cast<int>(costmap->getSizeInCellsX()); x++) {
          const bool in_mask = lethal_x >= 0 && lethal_y >= 0 &&
            x >= lethal_x && x < lethal_x + 3 && y >= lethal_y && y < lethal_y + 3;
          EXPECT_EQ(
            costmap->getCost(x, y),
            in_mask ? nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE) <<
            "x=" << x << " y=" << y;
        }
      }
    };

  // 1. The far mask at (40,40) received before the first update must be applied by that update
  layers_->updateMap(0.0, 0.0, 0.0);
  verify(40, 40);

  // 2. Replace it by an all-free mask at another place (10,10): the next update must
  //    also refresh the old (40,40) area, so previously lethal cells are cleared
  createFarMask(nav2_util::OCC_GRID_FREE, 10.0, 10.0);
  rePublishMask();
  layers_->updateMap(0.0, 0.0, 0.0);
  verify(-1, -1);

  // 3. Publish a lethal mask at a new far place (20,30): applied on the next update
  createFarMask(nav2_util::OCC_GRID_OCCUPIED, 20.0, 30.0);
  rePublishMask();
  layers_->updateMap(0.0, 0.0, 0.0);
  verify(20, 30);
}

void TestNode::reset()
{
  layers_.reset();
  mask_.reset();
  master_grid_.reset();
  info_publisher_.reset();
  mask_publisher_.reset();
  keepout_filter_.reset();
  node_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
  keepout_points_.clear();
}

TEST_F(TestNode, testFreeMasterLethalKeepout)
{
  // Initialize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_OCCUPIED, "map");
  publishMaps();
  createKeepoutFilter("map");

  // Test KeepoutFilter
  testStandardScenario(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testUnknownMasterNonLethalKeepout)
{
  // Initialize test system
  createMaps(
    nav2_costmap_2d::NO_INFORMATION,
    (nav2_util::OCC_GRID_OCCUPIED - nav2_util::OCC_GRID_FREE) / 2,
    "map");
  publishMaps();
  createKeepoutFilter("map");

  // Test KeepoutFilter
  testStandardScenario(
    nav2_costmap_2d::NO_INFORMATION,
    (nav2_costmap_2d::LETHAL_OBSTACLE - nav2_costmap_2d::FREE_SPACE) / 2);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testFreeKeepout)
{
  // Initialize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_FREE, "map");
  publishMaps();
  createKeepoutFilter("map");

  // Test KeepoutFilter
  geometry_msgs::msg::Pose2D pose;
  // Check whole area window
  keepout_filter_->process(*master_grid_, 0, 0, 10, 10, pose);
  // There should be no one point appeared on master_grid_ after process()
  verifyMasterGrid(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testUnknownKeepout)
{
  // Initialize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_UNKNOWN, "map");
  publishMaps();
  createKeepoutFilter("map");

  // Test KeepoutFilter
  geometry_msgs::msg::Pose2D pose;
  // Check whole area window
  keepout_filter_->process(*master_grid_, 0, 0, 10, 10, pose);
  // There should be no one point appeared on master_grid_ after process()
  verifyMasterGrid(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testInfoRePublish)
{
  // Initialize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_OCCUPIED, "map");
  publishMaps();
  createKeepoutFilter("map");

  // Re-publish filter info (with incorrect base and multiplier)
  // and test that everything is working after
  rePublishInfo(0.1, 0.2);

  // Test KeepoutFilter
  testStandardScenario(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testMaskRePublish)
{
  // Initialize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_OCCUPIED, "map");
  publishMaps();
  createKeepoutFilter("map");

  // Re-publish filter mask and test that everything is working after
  rePublishMask();

  // Test KeepoutFilter
  testStandardScenario(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testDifferentFrames)
{
  // Initialize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_OCCUPIED, "map");
  publishMaps();
  createKeepoutFilter("odom");
  createTFBroadcaster("map", "odom");

  // Test KeepoutFilter
  testFramesScenario(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testFarMaskUpdateBounds)
{
  // Initialize test system
  createFarMask(nav2_util::OCC_GRID_OCCUPIED, 40.0, 40.0);
  publishMaps();
  createKeepoutFilter("map");

  // Test KeepoutFilter update window for a small mask far from the master origin
  testFarMaskScenario();

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
