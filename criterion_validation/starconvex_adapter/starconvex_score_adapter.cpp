#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "CorridorBuilder.h"
#include "MeshTable.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Options {
  std::string input;
  std::string output;
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
  double sample_xy = 0.25;
  double sample_z = 0.1;
};

struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::unordered_map<std::string, std::size_t> index;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Snapshot {
  std::string id;
  std::vector<std::size_t> row_indices;
  std::vector<Vec3> positions;
  std::vector<bool> position_seen;
  double pillar_y = 0.0;
  double pillar_radius = 0.0;
};

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::string current;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        current.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      fields.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  if (quoted) throw std::runtime_error("unterminated quoted CSV field");
  fields.push_back(current);
  return fields;
}

CsvTable read_csv(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open input CSV: " + path);
  CsvTable table;
  std::string line;
  if (!std::getline(stream, line)) throw std::runtime_error("empty input CSV");
  if (!line.empty() && line.back() == '\r') line.pop_back();
  table.header = split_csv_line(line);
  for (std::size_t i = 0; i < table.header.size(); ++i) {
    table.index.emplace(table.header[i], i);
  }
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    auto row = split_csv_line(line);
    if (row.size() != table.header.size()) {
      throw std::runtime_error("CSV row has unexpected field count");
    }
    table.rows.push_back(std::move(row));
  }
  return table;
}

const std::string& field(const CsvTable& table,
                         const std::vector<std::string>& row,
                         const std::string& name) {
  const auto found = table.index.find(name);
  if (found == table.index.end()) throw std::runtime_error("missing CSV field: " + name);
  return row.at(found->second);
}

double number(const CsvTable& table,
              const std::vector<std::string>& row,
              const std::string& name) {
  return std::stod(field(table, row, name));
}

int integer(const CsvTable& table,
            const std::vector<std::string>& row,
            const std::string& name) {
  return std::stoi(field(table, row, name));
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (i + 1 >= argc) throw std::runtime_error("missing value for " + key);
    const std::string value(argv[++i]);
    if (key == "--input") options.input = value;
    else if (key == "--output") options.output = value;
    else if (key == "--sensor-range") options.sensor_range = std::stod(value);
    else if (key == "--azimuth-step") options.azimuth_step = std::stod(value);
    else if (key == "--raycast-resolution") options.raycast_resolution = std::stod(value);
    else if (key == "--vertical-resolution") options.vertical_resolution = std::stod(value);
    else if (key == "--frontier-floor") options.frontier_floor = std::stod(value);
    else if (key == "--frontier-roof") options.frontier_roof = std::stod(value);
    else if (key == "--delta-floor") options.delta_floor = std::stod(value);
    else if (key == "--delta-roof") options.delta_roof = std::stod(value);
    else if (key == "--extension-distance-threshold") options.extension_distance_threshold = std::stod(value);
    else if (key == "--extension-angle-threshold") options.extension_angle_threshold = std::stod(value);
    else if (key == "--sample-xy") options.sample_xy = std::stod(value);
    else if (key == "--sample-z") options.sample_z = std::stod(value);
    else throw std::runtime_error("unknown option: " + key);
  }
  if (options.input.empty() || options.output.empty()) {
    throw std::runtime_error("--input and --output are required");
  }
  if (options.sensor_range <= 0.0 || options.azimuth_step <= 0.0 ||
      options.raycast_resolution <= 0.0 ||
      options.vertical_resolution <= 0.0 || options.sample_xy <= 0.0 ||
      options.sample_z <= 0.0 || options.frontier_roof <= options.frontier_floor) {
    throw std::runtime_error("invalid non-positive geometry option");
  }
  return options;
}

Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 subtract(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 scale(const Vec3& value, double factor) { return {value.x * factor, value.y * factor, value.z * factor}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Vec3& value) { return std::sqrt(dot(value, value)); }
Vec3 normalized(const Vec3& value) {
  const double length = norm(value);
  if (length < 1e-12) throw std::runtime_error("cannot normalize zero vector");
  return scale(value, 1.0 / length);
}

Vec3 first_cylinder_hit(const Vec3& origin, const Vec3& endpoint,
                        double pillar_y, double pillar_radius,
                        double raycast_resolution) {
  const Vec3 delta = subtract(endpoint, origin);
  const double a = delta.x * delta.x + delta.y * delta.y;
  const double ox = origin.x;
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
      ray_length, std::ceil((hit_t * ray_length) / raycast_resolution) * raycast_resolution);
  return add(origin, scale(delta, sampled_distance / ray_length));
}

void extend_points(std::vector<Vec3>& points, const Vec3& first, const Vec3& second,
                   const Vec3& key, double range, double pillar_y,
                   double pillar_radius, const Options& options, int depth = 0) {
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
  const Vec3 direction_sum = add(normalized(first_relative), normalized(second_relative));
  if (norm(direction_sum) < 1e-12) {
    points.push_back(second);
    return;
  }
  const Vec3 endpoint = add(key, scale(normalized(direction_sum), range));
  const Vec3 middle = first_cylinder_hit(key, endpoint, pillar_y, pillar_radius,
                                         options.raycast_resolution);
  extend_points(points, first, middle, key, range, pillar_y, pillar_radius, options, depth + 1);
  extend_points(points, middle, second, key, range, pillar_y, pillar_radius, options, depth + 1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generate_boundary_cloud(
    const Vec3& key, double pillar_y, double pillar_radius, const Options& options) {
  std::vector<Vec3> boundary;
  std::vector<std::size_t> before_size{0};
  const std::array<double, 3> ranges{{
      options.sensor_range, options.sensor_range / 2.0, options.sensor_range / 4.0}};
  const double floor = std::max(key.z + options.delta_floor, options.frontier_floor);
  const double roof = std::min(key.z + options.delta_roof, options.frontier_roof);
  if (roof < floor) throw std::runtime_error("empty StarConvex vertical sampling interval");

  for (double range : ranges) {
    const std::size_t batch_start = boundary.size();
    for (double z = floor; z <= roof + 1e-9; z += options.vertical_resolution) {
      for (double theta = 0.0; theta < 2.0 * kPi; theta += options.azimuth_step) {
        const Vec3 endpoint{key.x + std::cos(theta) * range,
                            key.y + std::sin(theta) * range, z};
        const Vec3 hit = first_cylinder_hit(key, endpoint, pillar_y, pillar_radius,
                                            options.raycast_resolution);
        if (norm(subtract(hit, key)) < 0.3) continue;
        if (boundary.size() > batch_start) {
          extend_points(boundary, boundary.back(), hit, key, range,
                        pillar_y, pillar_radius, options);
        } else {
          boundary.push_back(hit);
        }
      }
    }
    before_size.push_back(boundary.size());
  }

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
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
    cloud->push_back(pcl::PointXYZ(boundary[i].x, boundary[i].y, boundary[i].z));
    maximum_z = std::max(maximum_z, boundary[i].z);
    minimum_z = std::min(minimum_z, boundary[i].z);
  }
  if (cloud->empty()) throw std::runtime_error("StarConvex boundary cloud is empty");
  cloud->push_back(pcl::PointXYZ(key.x, key.y, std::max(key.z + 1e-3, maximum_z)));
  cloud->push_back(pcl::PointXYZ(key.x, key.y, std::min(key.z - 1e-3, minimum_z)));
  return cloud;
}

class OriginalStarSpace {
 public:
  OriginalStarSpace(const Vec3& center, double pillar_y, double pillar_radius,
                    const Options& options)
      : center_(center), input_cloud_(generate_boundary_cloud(
            center, pillar_y, pillar_radius, options)) {
    build_two_pass();
  }

  bool contains(const Vec3& world) {
    const Vec3 local = subtract(world, center_);
    if (norm(local) < 1e-9) return true;
    pcl::PointXYZ query(local.x, local.y, local.z);
    pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
    return table_.isInStarHull(meshes_, origin, query) > 0.0;
  }

  std::size_t input_point_count() const { return input_cloud_->size(); }
  std::size_t mesh_count() const { return meshes_.size(); }

 private:
  void build_two_pass() {
    pcl::PointXYZ origin(center_.x, center_.y, center_.z);
    std::vector<FastLab::Triangle> first_meshes;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::size_t> indices;
    FastLab::Corridor first_builder(input_cloud_, 500.0f, 0.02f);
    first_builder.build(first_meshes, vertices, indices, origin, true);
    if (vertices.empty()) throw std::runtime_error("first StarConvex pass returned no vertices");

    auto second_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& vertex : vertices) {
      second_cloud->push_back(pcl::PointXYZ(vertex.x(), vertex.y(), vertex.z()));
    }
    vertices.clear();
    indices.clear();
    FastLab::Corridor second_builder(second_cloud, 500.0f, 0.02f);
    second_builder.build(meshes_, vertices, indices, origin, true);
    if (meshes_.empty()) throw std::runtime_error("second StarConvex pass returned no meshes");
    for (auto& mesh : meshes_) {
      mesh._a.x -= center_.x; mesh._a.y -= center_.y; mesh._a.z -= center_.z;
      mesh._b.x -= center_.x; mesh._b.y -= center_.y; mesh._b.z -= center_.z;
      mesh._c.x -= center_.x; mesh._c.y -= center_.y; mesh._c.z -= center_.z;
      mesh.calcuNormal();
    }
    table_.createTable(meshes_);
  }

  Vec3 center_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  std::vector<FastLab::Triangle> meshes_;
  FastLab::MeshTable table_;
};

std::map<std::string, Snapshot> build_snapshots(const CsvTable& table) {
  int maximum_uav_id = -1;
  for (const auto& row : table.rows) {
    maximum_uav_id = std::max(maximum_uav_id, integer(table, row, "uav_i"));
    maximum_uav_id = std::max(maximum_uav_id, integer(table, row, "uav_j"));
  }
  if (maximum_uav_id < 1) throw std::runtime_error("at least two UAV ids are required");
  const std::size_t num_uavs = static_cast<std::size_t>(maximum_uav_id + 1);
  std::map<std::string, Snapshot> snapshots;
  for (std::size_t row_index = 0; row_index < table.rows.size(); ++row_index) {
    const auto& row = table.rows[row_index];
    const std::string id = field(table, row, "snapshot_id");
    auto inserted = snapshots.emplace(id, Snapshot{});
    Snapshot& snapshot = inserted.first->second;
    if (inserted.second) {
      snapshot.id = id;
      snapshot.positions.resize(num_uavs);
      snapshot.position_seen.assign(num_uavs, false);
      snapshot.pillar_y = number(table, row, "pillar_y");
      snapshot.pillar_radius = number(table, row, "inflated_pillar_radius");
    }
    snapshot.row_indices.push_back(row_index);
    const double z = number(table, row, "center_z");
    for (const char suffix : {'i', 'j'}) {
      const std::string suffix_text(1, suffix);
      const int uav = integer(table, row, "uav_" + suffix_text);
      if (uav < 0 || static_cast<std::size_t>(uav) >= num_uavs) {
        throw std::runtime_error("UAV id is outside inferred contiguous range");
      }
      const Vec3 position{number(table, row, "uav_" + suffix_text + "_x"),
                          number(table, row, "uav_" + suffix_text + "_y"), z};
      if (snapshot.position_seen[uav]) {
        const Vec3 difference = subtract(snapshot.positions[uav], position);
        if (norm(difference) > 1e-9) throw std::runtime_error("inconsistent UAV position");
      } else {
        snapshot.positions[uav] = position;
        snapshot.position_seen[uav] = true;
      }
    }
  }
  for (const auto& entry : snapshots) {
    if (std::find(entry.second.position_seen.begin(), entry.second.position_seen.end(), false)
        != entry.second.position_seen.end()) {
      throw std::runtime_error("snapshot does not expose all 8 UAV positions");
    }
  }
  return snapshots;
}

std::string quote_csv(const std::string& value) {
  if (value.find_first_of(",\"") == std::string::npos) return value;
  std::string escaped = "\"";
  for (char c : value) escaped += (c == '"' ? "\"\"" : std::string(1, c));
  return escaped + "\"";
}

std::string format_number(double value) {
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

void write_row(std::ofstream& output, const std::vector<std::string>& original,
               const std::vector<std::string>& appended) {
  bool first = true;
  for (const auto& value : original) {
    if (!first) output << ',';
    output << quote_csv(value);
    first = false;
  }
  for (const auto& value : appended) output << ',' << quote_csv(value);
  output << '\n';
}

int run(const Options& options) {
  CsvTable table = read_csv(options.input);
  const auto snapshots = build_snapshots(table);
  std::ofstream output(options.output);
  if (!output) throw std::runtime_error("cannot open output CSV: " + options.output);
  const std::vector<std::string> appended_header{
      "starconvex_input_points_i", "starconvex_input_points_j",
      "starconvex_meshes_i", "starconvex_meshes_j",
      "real_volume_i", "real_volume_j", "real_intersection_volume",
      "real_union_volume", "real_overlap_min", "real_overlap_iou",
      "real_composite_min", "real_composite_iou"};
  write_row(output, table.header, appended_header);

  std::size_t completed = 0;
  for (const auto& entry : snapshots) {
    const Snapshot& snapshot = entry.second;
    const std::size_t num_uavs = snapshot.positions.size();
    std::vector<std::unique_ptr<OriginalStarSpace>> spaces(num_uavs);
    for (std::size_t uav = 0; uav < num_uavs; ++uav) {
      spaces[uav].reset(new OriginalStarSpace(snapshot.positions[uav], snapshot.pillar_y,
                                               snapshot.pillar_radius, options));
    }

    double minimum_x = snapshot.positions[0].x;
    double maximum_x = snapshot.positions[0].x;
    double minimum_y = snapshot.positions[0].y;
    double maximum_y = snapshot.positions[0].y;
    for (const auto& position : snapshot.positions) {
      minimum_x = std::min(minimum_x, position.x);
      maximum_x = std::max(maximum_x, position.x);
      minimum_y = std::min(minimum_y, position.y);
      maximum_y = std::max(maximum_y, position.y);
    }
    minimum_x -= options.sensor_range;
    maximum_x += options.sensor_range;
    minimum_y -= options.sensor_range;
    maximum_y += options.sensor_range;
    const double minimum_z = std::max(snapshot.positions[0].z + options.delta_floor,
                                      options.frontier_floor);
    const double maximum_z = std::min(snapshot.positions[0].z + options.delta_roof,
                                      options.frontier_roof);
    const int count_x = static_cast<int>(std::ceil((maximum_x - minimum_x) / options.sample_xy));
    const int count_y = static_cast<int>(std::ceil((maximum_y - minimum_y) / options.sample_xy));
    const int count_z = std::max(1, static_cast<int>(std::ceil((maximum_z - minimum_z) / options.sample_z)));
    std::vector<std::size_t> volume_counts(num_uavs, 0);
    std::map<std::pair<int, int>, std::size_t> intersection_counts;
    for (std::size_t row_index : snapshot.row_indices) {
      const auto& row = table.rows[row_index];
      intersection_counts.emplace(std::make_pair(integer(table, row, "uav_i"),
                                                  integer(table, row, "uav_j")), 0);
    }

    for (int ix = 0; ix < count_x; ++ix) {
      const double x = minimum_x + (ix + 0.5) * options.sample_xy;
      for (int iy = 0; iy < count_y; ++iy) {
        const double y = minimum_y + (iy + 0.5) * options.sample_xy;
        for (int iz = 0; iz < count_z; ++iz) {
          const double z = minimum_z + (iz + 0.5) * options.sample_z;
          std::vector<bool> membership(num_uavs, false);
          for (std::size_t uav = 0; uav < num_uavs; ++uav) {
            membership[uav] = spaces[uav]->contains({x, y, z});
            volume_counts[uav] += static_cast<std::size_t>(membership[uav]);
          }
          for (auto& pair_count : intersection_counts) {
            pair_count.second += static_cast<std::size_t>(
                membership[pair_count.first.first] && membership[pair_count.first.second]);
          }
        }
      }
    }

    const double cell_volume = options.sample_xy * options.sample_xy * options.sample_z;
    for (std::size_t row_index : snapshot.row_indices) {
      const auto& row = table.rows[row_index];
      const int first = integer(table, row, "uav_i");
      const int second = integer(table, row, "uav_j");
      const std::size_t count_i = volume_counts[first];
      const std::size_t count_j = volume_counts[second];
      const std::size_t intersection = intersection_counts.at({first, second});
      const std::size_t union_count = count_i + count_j - intersection;
      const std::size_t smaller = std::min(count_i, count_j);
      const double overlap_min = smaller ? static_cast<double>(intersection) / smaller : 0.0;
      const double overlap_iou = union_count ? static_cast<double>(intersection) / union_count : 0.0;
      const double weight = number(table, row, "visibility_weight");
      const std::vector<std::string> appended{
          std::to_string(spaces[first]->input_point_count()),
          std::to_string(spaces[second]->input_point_count()),
          std::to_string(spaces[first]->mesh_count()),
          std::to_string(spaces[second]->mesh_count()),
          format_number(count_i * cell_volume), format_number(count_j * cell_volume),
          format_number(intersection * cell_volume), format_number(union_count * cell_volume),
          format_number(overlap_min), format_number(overlap_iou),
          format_number(overlap_min * weight), format_number(overlap_iou * weight)};
      write_row(output, row, appended);
    }
    ++completed;
    std::cerr << "StarConvex snapshot " << completed << '/' << snapshots.size()
              << " complete (id=" << snapshot.id << ")\n";
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(parse_options(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "starconvex_score_adapter: " << error.what() << '\n';
    return 1;
  }
}
