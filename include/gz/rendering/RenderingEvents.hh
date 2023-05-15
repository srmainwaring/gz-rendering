/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GZ_RENDERING_RENDERINGEVENTS_HH_
#define GZ_RENDERING_RENDERINGEVENTS_HH_

#include <string>
#include <vector>

#include <gz/common/Event.hh>
#include <gz/common/EventFactory.hh>

#include "gz/rendering/BoundingBox.hh"

namespace gz
{
namespace rendering
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_RENDERING_VERSION_NAMESPACE {
/// \brief Namespace for all events.
namespace events
{
  // BaseCamera
  using NewFrameEvent = common::EventT<void(
      const void *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewFrameEventTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewFrameEvent",
      NewFrameEvent)

  // OgreDepthCamera
  using NewOgreRgbPointCloud = gz::common::EventT<void(
      const float *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgreRgbPointCloudTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgreRgbPointCloud",
      NewOgreRgbPointCloud)

  using NewOgreDepthFrame = gz::common::EventT<void(
      const float *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgreDepthFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgreDepthFrame",
      NewOgreDepthFrame)

  // OgreGpuRays
  using NewOgreGpuRaysFrame = gz::common::EventT<void(
      const float *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgreGpuRaysFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgreGpuRaysFrame",
      NewOgreGpuRaysFrame)

  // OgreThermalCamera
  using NewOgreThermalFrame = gz::common::EventT<void(
      const uint16_t *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgreThermalFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgreThermalFrame",
      NewOgreThermalFrame)

  // OgreWideAngleCamera
  using NewOgreWideAngleFrame = gz::common::EventT<void(
      const unsigned char *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgreWideAngleFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgreWideAngleFrame",
      NewOgreWideAngleFrame)

  // Ogre2BoundingBoxCamera
  using NewOgre2BoundingBoxes = gz::common::EventT<void(
      const std::vector<BoundingBox> &), struct NewOgre2BoundingBoxesTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgre2BoundingBoxes",
      NewOgre2BoundingBoxes)

  // Ogre2DepthCamera
  using NewOgre2RgbPointCloud = gz::common::EventT<void(
      const float *,
      unsigned int, unsigned int, unsigned int,
      const std::string &),struct NewOgre2RgbPointCloudTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgre2RgbPointCloud",
      NewOgre2RgbPointCloud)

  using NewOgre2DepthFrame = gz::common::EventT<void(
      const float *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgre2DepthFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgre2DepthFrame",
      NewOgre2DepthFrame)

  // Ogre2GpuRays
  using NewOgre2GpuRaysFrame = gz::common::EventT<void(
      const float *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgre2GpuRaysFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgre2GpuRaysFrame",
      NewOgre2GpuRaysFrame)

  // Ogre2SegmentationCamera
  using NewOgre2SegmentationFrame = gz::common::EventT<void(
      const uint8_t *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgre2SegmentationFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgre2SegmentationFrame",
      NewOgre2SegmentationFrame)

  // Ogre2ThermalCamera
  using NewOgre2ThermalFrame = gz::common::EventT<void(
      const uint16_t *,
      unsigned int, unsigned int, unsigned int,
      const std::string &), struct NewOgre2ThermalFrameTag>;
  GZ_COMMON_REGISTER_EVENT(
      "gz_rendering_events.NewOgre2ThermalFrame",
      NewOgre2ThermalFrame)

} // namespace events
}
}  // namespace rendering
}  // namespace gz
#endif  // GZ_RENDERING_RENDERINGEVENTS_HH_
