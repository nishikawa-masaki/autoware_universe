// Copyright 2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file cpu_usage_statistics.cpp
 * @brief CPU usage statistics collection and calculation implementation
 */

#include "system_monitor/cpu_monitor/cpu_usage_statistics.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>  // For uint64_t
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

CpuUsageStatistics::CpuUsageStatistics() : first_call_(true) {}

void CpuUsageStatistics::collect_cpu_stats(std::vector<CoreUsageInfo> & core_usage_info)
{
  // Read current CPU statistics from /proc/stat
  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    return;
  }
  
  std::string line;
  std::vector<CpuStats> current_stats;
  
  while (std::getline(stat_file, line)) {
    try {
      std::istringstream iss(line);
      std::string cpu_name;
      iss >> cpu_name;
      
      // Skip lines that don't start with "cpu"
      if (cpu_name.substr(0, 3) != "cpu") {
        continue;
      }
      if (cpu_name == "cpu") {
        cpu_name = "all";
      } else {
        cpu_name = cpu_name.substr(3);
      }
      
      CpuStats stats;
      iss >> stats.user >> stats.nice >> stats.system >> stats.idle 
        >> stats.iowait >> stats.irq >> stats.softirq >> stats.steal 
        >> stats.guest >> stats.guest_nice;
      
      stats.name = cpu_name;
      current_stats.push_back(stats);
    } catch (const std::exception& e) {
      // Log error but continue processing other lines
      continue;
    }
  }
  stat_file.close();
  
  // If this is the first call, just store the current stats and return empty info
  if (first_call_) {
    prev_stats_ = current_stats;
    first_call_ = false;
    return;
  }
  
  // Process each CPU's statistics
  for (const auto & core_info : current_stats) {
    const std::string & cpu_name = core_info.name;
    const CpuStats & stats = core_info;
    
    // Skip if we don't have previous stats for this CPU
    auto prev_iter = std::find_if(prev_stats_.begin(), prev_stats_.end(), [cpu_name](const CpuStats & stats) {
      return stats.name == cpu_name;
    });
    if (prev_iter == prev_stats_.end()) {
      continue;
    }
    
    // Calculate deltas
    uint64_t user_delta = stats.user - prev_iter->user;
    uint64_t nice_delta = stats.nice - prev_iter->nice;
    uint64_t system_delta = stats.system - prev_iter->system;
    uint64_t idle_delta = stats.idle - prev_iter->idle;
    uint64_t iowait_delta = stats.iowait - prev_iter->iowait;
    uint64_t irq_delta = stats.irq - prev_iter->irq;
    uint64_t softirq_delta = stats.softirq - prev_iter->softirq;
    uint64_t steal_delta = stats.steal - prev_iter->steal;
    uint64_t guest_delta = stats.guest - prev_iter->guest;
    uint64_t guest_nice_delta = stats.guest_nice - prev_iter->guest_nice;
    
    // Calculate total time delta
    uint64_t total_delta = user_delta + nice_delta + system_delta + 
                          iowait_delta + irq_delta + softirq_delta + steal_delta;
    uint64_t total_all_delta = total_delta + idle_delta;
    
    // Skip if total time delta is zero
    if (total_all_delta == 0) {
      continue;
    }

    float total_all_delta_float = static_cast<float>(total_all_delta);
    CoreUsageInfo core_usage;
    core_usage.name = cpu_name;
    core_usage.user_percent = (static_cast<float>(user_delta) / total_all_delta_float) * 100.0f;
    core_usage.nice_percent = (static_cast<float>(nice_delta) / total_all_delta_float) * 100.0f;
    core_usage.system_percent = (static_cast<float>(system_delta) / total_all_delta_float) * 100.0f;
    core_usage.idle_percent = (static_cast<float>(idle_delta) / total_all_delta_float) * 100.0f;
    core_usage.iowait_percent = (static_cast<float>(iowait_delta) / total_all_delta_float) * 100.0f;
    core_usage.irq_percent = (static_cast<float>(irq_delta) / total_all_delta_float) * 100.0f;
    core_usage.softirq_percent = (static_cast<float>(softirq_delta) / total_all_delta_float) * 100.0f;
    core_usage.steal_percent = (static_cast<float>(steal_delta) / total_all_delta_float) * 100.0f;
    core_usage.guest_percent = (static_cast<float>(guest_delta) / total_all_delta_float) * 100.0f;
    core_usage.gnice_percent = (static_cast<float>(guest_nice_delta) / total_all_delta_float) * 100.0f;
    core_usage.total_usage_percent = 100.0f - core_usage.idle_percent;

    // Add to CPU info
    core_usage_info.push_back(core_usage);
  }
  
  // Store current stats for next call
  prev_stats_ = current_stats;
}