#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "armor/types.hpp"

namespace armor {

struct PairLB {
  int li = -1;   // left index  (x 更小)
  int ri = -1;   // right index
  double score = 0.0;
};

std::vector<PairLB> PairLights(const std::vector<LBBox>& bars, const YAML::Node& P);

} // namespace armor
