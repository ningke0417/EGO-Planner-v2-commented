#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "CorridorBuilder.h"
#include "MeshTable.hpp"

namespace starconvex_adapter {

constexpr double kPi = 3.14159265358979323846;

struct GeometryOptions {
  double sensor_range = 5.0;
  double azimuth_step = 0.1;
  double raycast_resolution = 0.1;
  double vertical_resolution = 0.2;
  double frontier_floor = 0.5;
  double frontier_roof = 1.5;
  double delta_floor = -0.5;
  double delta_roof = 1.5;
  double extension_distance_threshold = 1.0;
  double extension_angle_threshold = 0.1;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Vec3 add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 subtract(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 scale(const Vec3& value, double factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double norm(const Vec3& value) { return std::sqrt(dot(value, value)); }

inline Vec3 normalized(const Vec3& value) {
  const double length = norm(value);
  if (length < 1e-12) throw std::runtime_error("cannot normalize zero vector");
  return scale(value, 1.0 / length);
}

inline Vec3 firstCylinderHit(const Vec3& origin, const Vec3& endpoint,
                             double pillar_x, double pillar_y,
                             double pillar_radius,
                             double raycast_resolution) {
  const Vec3 delta = subtract(endpoint, origin);
  const double a = delta.x * delta.x + delta.y * delta.y;
  const double ox = origin.x - pillar_x;
  const double oy = origin.y - pillar_y;
  const double c = ox * ox + oy * oy - pillar_radius * pillar_radius;
  if (c <= 0.0 || a < 1e-12) return origin;
  const double b = 2.0 * (ox * delta.x + oy * delta.y);
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) return endpoint;
  const double root = std::sqrt(discriminant);
  const double near_t = (-b - root) / (2.0 * a);
  const double far_t = (-b + root) / (2.0 * a);
  double hit_t = std::numeric_limits<double>::infinity();
  if (near_t >= 0.0 && near_t <= 1.0) hit_t = near_t;
  else if (far_t >= 0.0 && far_t <= 1.0) hit_t = far_t;
  if (!std::isfinite(hit_t)) return endpoint;
  const double ray_length = norm(delta);
  const double sampled_distance = std::min(
      ray_length,
      std::ceil(hit_t * ray_length / raycast_resolution) * raycast_resolution);
  return add(origin, scale(delta, sampled_distance / ray_length));
}

inline void extendPoints(std::vector<Vec3>& points, const Vec3& first,
                         const Vec3& second, const Vec3& key, double range,
                         double pillar_x, double pillar_y,
                         double pillar_radius, const GeometryOptions& options,
                         int depth = 0) {
  const Vec3 first_relative = subtract(first, key);
  const Vec3 second_relative = subtract(second, key);
  const double r1 = norm(first_relative);
  const double r2 = norm(second_relative);
  const double separation = norm(subtract(first, second));
  if (depth >= 20 || separation < options.extension_distance_threshold ||
      r1 < 1e-12 || r2 < 1e-12) {
    points.push_back(second);
    return;
  }
  double cosine = dot(first_relative, second_relative) / (r1 * r2);
  cosine = std::max(-1.0, std::min(1.0, cosine));
  const double angle = std::acos(cosine);
  if (std::sin(angle) * std::min(r1, r2) / separation <
      std::sin(options.extension_angle_threshold)) {
    points.push_back(second);
    return;
  }
  const Vec3 direction_sum =
      add(normalized(first_relative), normalized(second_relative));
  if (norm(direction_sum) < 1e-12) {
    points.push_back(second);
    return;
  }
  const Vec3 endpoint = add(key, scale(normalized(direction_sum), range));
  const Vec3 middle = firstCylinderHit(
      key, endpoint, pillar_x, pillar_y, pillar_radius,
      options.raycast_resolution);
  extendPoints(points, first, middle, key, range, pillar_x, pillar_y,
               pillar_radius, options, depth + 1);
  extendPoints(points, middle, second, key, range, pillar_x, pillar_y,
               pillar_radius, options, depth + 1);
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr generateBoundaryCloud(
    const Vec3& key, double pillar_x, double pillar_y, double pillar_radius,
    const GeometryOptions& options) {
  std::vector<Vec3> boundary;
  std::vector<std::size_t> before_size{0};
  const std::array<double, 3> ranges{{options.sensor_range,
                                      options.sensor_range / 2.0,
                                      options.sensor_range / 4.0}};
  const double floor =
      std::max(key.z + options.delta_floor, options.frontier_floor);
  const double roof =
      std::min(key.z + options.delta_roof, options.frontier_roof);
  if (roof < floor) {
    throw std::runtime_error("empty StarConvex vertical sampling interval");
  }

  for (double range : ranges) {
    const std::size_t batch_start = boundary.size();
    for (double z = floor; z <= roof + 1e-9;
         z += options.vertical_resolution) {
      for (double theta = 0.0; theta < 2.0 * kPi;
           theta += options.azimuth_step) {
        const Vec3 endpoint{key.x + std::cos(theta) * range,
                            key.y + std::sin(theta) * range, z};
        const Vec3 hit = firstCylinderHit(
            key, endpoint, pillar_x, pillar_y, pillar_radius,
            options.raycast_resolution);
        if (norm(subtract(hit, key)) < 0.3) continue;
        if (boundary.size() > batch_start) {
          extendPoints(boundary, boundary.back(), hit, key, range, pillar_x,
                       pillar_y, pillar_radius, options);
        } else {
          boundary.push_back(hit);
        }
      }
    }
    before_size.push_back(boundary.size());
  }

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  double maximum_z = -std::numeric_limits<double>::infinity();
  double minimum_z = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < boundary.size(); ++i) {
    bool discard_inner_max_range = false;
    for (std::size_t batch = 1; batch < before_size.size(); ++batch) {
      if (i >= before_size[batch] &&
          norm(subtract(boundary[i], key)) > ranges[batch] - 0.5) {
        discard_inner_max_range = true;
        break;
      }
    }
    if (discard_inner_max_range) continue;
    cloud->push_back(
        pcl::PointXYZ(boundary[i].x, boundary[i].y, boundary[i].z));
    maximum_z = std::max(maximum_z, boundary[i].z);
    minimum_z = std::min(minimum_z, boundary[i].z);
  }
  if (cloud->empty()) {
    throw std::runtime_error("StarConvex boundary cloud is empty");
  }
  cloud->push_back(
      pcl::PointXYZ(key.x, key.y, std::max(key.z + 1e-3, maximum_z)));
  cloud->push_back(
      pcl::PointXYZ(key.x, key.y, std::min(key.z - 1e-3, minimum_z)));
  return cloud;
}

struct VoxelIndex {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelIndex& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash {
  std::size_t operator()(const VoxelIndex& index) const {
    std::size_t seed = std::hash<std::int64_t>()(index.x);
    seed ^= std::hash<std::int64_t>()(index.y) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>()(index.z) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
    return seed;
  }
};

// Immutable occupancy field built from the same global point cloud consumed by
// the planners. Inflation is performed once when the latched map arrives.
class VoxelObstacleField {
 public:
  VoxelObstacleField(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                     double resolution, double inflation)
      : resolution_(resolution) {
    if (resolution <= 0.0 || inflation < 0.0)
      throw std::runtime_error("invalid voxel obstacle field parameters");

    std::unordered_set<VoxelIndex, VoxelIndexHash> raw;
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z))
        continue;
      raw.insert(indexOf({point.x, point.y, point.z}));
    }
    if (raw.empty()) throw std::runtime_error("point-cloud obstacle field is empty");

    const int radius = static_cast<int>(std::ceil(inflation / resolution_));
    std::vector<VoxelIndex> offsets;
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          const double distance =
              resolution_ * std::sqrt(dx * dx + dy * dy + dz * dz);
          if (distance <= inflation + 0.5 * resolution_ + 1e-9)
            offsets.push_back({dx, dy, dz});
        }
      }
    }
    if (offsets.empty()) offsets.push_back({0, 0, 0});
    occupied_.reserve(raw.size() * 2);
    for (const auto& index : raw) {
      for (const auto& offset : offsets) {
        occupied_.insert(
            {index.x + offset.x, index.y + offset.y, index.z + offset.z});
      }
    }
  }

  bool occupied(const Vec3& point) const {
    return occupied_.count(indexOf(point)) != 0;
  }

  Vec3 firstHit(const Vec3& origin, const Vec3& endpoint,
                double step) const {
    if (step <= 0.0) throw std::runtime_error("ray step must be positive");
    if (occupied(origin)) return origin;
    const Vec3 delta = subtract(endpoint, origin);
    const double length = norm(delta);
    if (length < 1e-12) return endpoint;
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = std::min(1.0, sample * step / length);
      const Vec3 point = add(origin, scale(delta, ratio));
      if (occupied(point)) return point;
    }
    return endpoint;
  }

  std::size_t occupiedVoxelCount() const { return occupied_.size(); }

 private:
  VoxelIndex indexOf(const Vec3& point) const {
    return {static_cast<std::int64_t>(std::llround(point.x / resolution_)),
            static_cast<std::int64_t>(std::llround(point.y / resolution_)),
            static_cast<std::int64_t>(std::llround(point.z / resolution_))};
  }

  double resolution_ = 0.1;
  std::unordered_set<VoxelIndex, VoxelIndexHash> occupied_;
};

using RaycastFunction = std::function<Vec3(const Vec3&, const Vec3&)>;

inline void extendPointsWithRaycast(std::vector<Vec3>& points,
                                    const Vec3& first, const Vec3& second,
                                    const Vec3& key, double range,
                                    const GeometryOptions& options,
                                    const RaycastFunction& raycast,
                                    int depth = 0) {
  const Vec3 first_relative = subtract(first, key);
  const Vec3 second_relative = subtract(second, key);
  const double r1 = norm(first_relative);
  const double r2 = norm(second_relative);
  const double separation = norm(subtract(first, second));
  if (depth >= 20 || separation < options.extension_distance_threshold ||
      r1 < 1e-12 || r2 < 1e-12) {
    points.push_back(second);
    return;
  }
  double cosine = dot(first_relative, second_relative) / (r1 * r2);
  cosine = std::max(-1.0, std::min(1.0, cosine));
  const double angle = std::acos(cosine);
  if (std::sin(angle) * std::min(r1, r2) / separation <
      std::sin(options.extension_angle_threshold)) {
    points.push_back(second);
    return;
  }
  const Vec3 direction_sum =
      add(normalized(first_relative), normalized(second_relative));
  if (norm(direction_sum) < 1e-12) {
    points.push_back(second);
    return;
  }
  const Vec3 endpoint = add(key, scale(normalized(direction_sum), range));
  const Vec3 middle = raycast(key, endpoint);
  extendPointsWithRaycast(points, first, middle, key, range, options, raycast,
                          depth + 1);
  extendPointsWithRaycast(points, middle, second, key, range, options, raycast,
                          depth + 1);
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr generateBoundaryCloud(
    const Vec3& key, const VoxelObstacleField& field,
    const GeometryOptions& options) {
  const RaycastFunction raycast = [&field, &options](const Vec3& origin,
                                                     const Vec3& endpoint) {
    return field.firstHit(origin, endpoint, options.raycast_resolution);
  };
  std::vector<Vec3> boundary;
  std::vector<std::size_t> before_size{0};
  const std::array<double, 3> ranges{{options.sensor_range,
                                      options.sensor_range / 2.0,
                                      options.sensor_range / 4.0}};
  const double floor =
      std::max(key.z + options.delta_floor, options.frontier_floor);
  const double roof =
      std::min(key.z + options.delta_roof, options.frontier_roof);
  if (roof < floor)
    throw std::runtime_error("empty StarConvex vertical sampling interval");

  for (double range : ranges) {
    const std::size_t batch_start = boundary.size();
    for (double z = floor; z <= roof + 1e-9;
         z += options.vertical_resolution) {
      for (double theta = 0.0; theta < 2.0 * kPi;
           theta += options.azimuth_step) {
        const Vec3 endpoint{key.x + std::cos(theta) * range,
                            key.y + std::sin(theta) * range, z};
        const Vec3 hit = raycast(key, endpoint);
        if (norm(subtract(hit, key)) < 0.3) continue;
        if (boundary.size() > batch_start) {
          extendPointsWithRaycast(boundary, boundary.back(), hit, key, range,
                                  options, raycast);
        } else {
          boundary.push_back(hit);
        }
      }
    }
    before_size.push_back(boundary.size());
  }

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  double maximum_z = -std::numeric_limits<double>::infinity();
  double minimum_z = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < boundary.size(); ++i) {
    bool discard_inner_max_range = false;
    for (std::size_t batch = 1; batch < before_size.size(); ++batch) {
      if (i >= before_size[batch] &&
          norm(subtract(boundary[i], key)) > ranges[batch] - 0.5) {
        discard_inner_max_range = true;
        break;
      }
    }
    if (discard_inner_max_range) continue;
    cloud->push_back(
        pcl::PointXYZ(boundary[i].x, boundary[i].y, boundary[i].z));
    maximum_z = std::max(maximum_z, boundary[i].z);
    minimum_z = std::min(minimum_z, boundary[i].z);
  }
  if (cloud->empty())
    throw std::runtime_error("point-cloud StarConvex boundary is empty");
  cloud->push_back(
      pcl::PointXYZ(key.x, key.y, std::max(key.z + 1e-3, maximum_z)));
  cloud->push_back(
      pcl::PointXYZ(key.x, key.y, std::min(key.z - 1e-3, minimum_z)));
  return cloud;
}

class OriginalStarSpace {
 public:
  OriginalStarSpace(const Vec3& center, double pillar_x, double pillar_y,
                    double pillar_radius, const GeometryOptions& options)
      : center_(center),
        input_cloud_(generateBoundaryCloud(center, pillar_x, pillar_y,
                                           pillar_radius, options)) {
    buildTwoPass();
  }

  OriginalStarSpace(const Vec3& center,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& boundary_cloud)
      : center_(center), input_cloud_(boundary_cloud) {
    if (!input_cloud_ || input_cloud_->empty())
      throw std::runtime_error("StarConvex boundary cloud is empty");
    buildTwoPass();
  }

  bool contains(const Vec3& world) {
    const Vec3 local = subtract(world, center_);
    if (norm(local) < 1e-9) return true;
    pcl::PointXYZ query(local.x, local.y, local.z);
    pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
    return table_.isInStarHull(meshes_, origin, query) > 0.0;
  }

  std::size_t inputPointCount() const { return input_cloud_->size(); }
  std::size_t meshCount() const { return meshes_.size(); }

 private:
  void buildTwoPass() {
    pcl::PointXYZ origin(center_.x, center_.y, center_.z);
    std::vector<FastLab::Triangle> first_meshes;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::size_t> indices;
    FastLab::Corridor first_builder(input_cloud_, 500.0f, 0.02f);
    first_builder.build(first_meshes, vertices, indices, origin, true);
    if (vertices.empty()) {
      throw std::runtime_error("first StarConvex pass returned no vertices");
    }

    auto second_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& vertex : vertices) {
      second_cloud->push_back(
          pcl::PointXYZ(vertex.x(), vertex.y(), vertex.z()));
    }
    vertices.clear();
    indices.clear();
    FastLab::Corridor second_builder(second_cloud, 500.0f, 0.02f);
    second_builder.build(meshes_, vertices, indices, origin, true);
    if (meshes_.empty()) {
      throw std::runtime_error("second StarConvex pass returned no meshes");
    }
    for (auto& mesh : meshes_) {
      mesh._a.x -= center_.x;
      mesh._a.y -= center_.y;
      mesh._a.z -= center_.z;
      mesh._b.x -= center_.x;
      mesh._b.y -= center_.y;
      mesh._b.z -= center_.z;
      mesh._c.x -= center_.x;
      mesh._c.y -= center_.y;
      mesh._c.z -= center_.z;
      mesh.calcuNormal();
    }
    table_.createTable(meshes_);
  }

  Vec3 center_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  std::vector<FastLab::Triangle> meshes_;
  FastLab::MeshTable table_;
};

}  // namespace starconvex_adapter
