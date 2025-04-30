// Copyright 2020,2025 Autoware Foundation
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

/**
 * @file cpu_monitor_base.cpp
 * @brief CPU monitor base class
 */

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

#include "system_monitor/cpu_monitor/cpu_information.hpp"
#include "system_monitor/cpu_monitor/cpu_usage_statistics.hpp"
#include "system_monitor/system_monitor_utility.hpp"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <chrono>  // for 1s
#include <cstdio>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace fs = boost::filesystem;

CPUMonitorBase::CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  hostname_(),
  num_cores_(0),
  temperatures_(),
  frequencies_(),
  usage_warn_(declare_parameter<float>(
    "usage_warn", 0.96,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
      "Threshold for CPU usage warning. Cannot be changed after initialization."))),
  usage_error_(declare_parameter<float>(
    "usage_error", 0.96,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
      "Threshold for CPU usage error. Cannot be changed after initialization."))),
  usage_warn_count_(declare_parameter<int>(
    "usage_warn_count", 1,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
      "Consecutive count threshold for CPU usage warning. Cannot be changed after "
      "initialization."))),
  usage_error_count_(declare_parameter<int>(
    "usage_error_count", 2,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
      "Consecutive count threshold for CPU usage error. Cannot be changed after initialization."))),
  usage_average_(declare_parameter<bool>(
    "usage_avg", true,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description(
      "Use average CPU usage across all processors. Cannot be changed after initialization.")))
{
  gethostname(hostname_, sizeof(hostname_));
  num_cores_ = boost::thread::hardware_concurrency();
  usage_warn_check_count_.resize(num_cores_ + 2);   // 2 = all + dummy
  usage_error_check_count_.resize(num_cores_ + 2);  // 2 = all + dummy

  updater_.setHardwareID(hostname_);
  // Update diagnostic data collected by the timer callback.
  updater_.add("CPU Temperature", this, &CPUMonitorBase::updateTemperature);
  updater_.add("CPU Usage", this, &CPUMonitorBase::updateUsage);
  updater_.add("CPU Load Average", this, &CPUMonitorBase::updateLoad);
  updater_.add("CPU Frequency", this, &CPUMonitorBase::updateFrequency);
  // Data format of ThermalThrottling differs among platforms.
  // So checking of status and updating of diagnostic are executed simultaneously.
  updater_.add("CPU Thermal Throttling", this, &CPUMonitorBase::checkThermalThrottling);

  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_cpu_usage_ =
    this->create_publisher<tier4_external_api_msgs::msg::CpuUsage>("~/cpu_usage", durable_qos);

  using namespace std::literals::chrono_literals;
  // Start timer for collecting cpu statistics
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&CPUMonitorBase::onTimer, this), timer_callback_group_);

  temperature_data_.clear();
  usage_data_.clear();
  load_data_.clear();
  frequency_data_.clear();
}

void CPUMonitorBase::update()
{
  updater_.force_update();
}

void CPUMonitorBase::checkTemperature()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  if (temperatures_.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    temperature_data_.clear();
    temperature_data_.summary_status = DiagStatus::ERROR;
    temperature_data_.summary_message = "temperature files not found";
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str = "";
  std::vector<TemperatureData::CoreTemperature> temporary_core_data{};

  for (const auto & entry : temperatures_) {
    // Read temperature file
    const fs::path path(entry.path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      error_str = "file open error";
      temporary_core_data.emplace_back(TemperatureData::CoreTemperature{
        entry.label_, DiagStatus::ERROR, 0.0f, error_str, entry.path_});
      continue;
    }

    float temperature;
    ifs >> temperature;
    ifs.close();
    temperature /= 1000;
    temporary_core_data.emplace_back(
      TemperatureData::CoreTemperature{entry.label_, DiagStatus::OK, temperature, "", ""});
  }

  std::lock_guard<std::mutex> lock(mutex_);
  temperature_data_.clear();
  if (!error_str.empty()) {
    temperature_data_.summary_status = DiagStatus::ERROR;
    temperature_data_.summary_message = error_str;
  } else {
    temperature_data_.core_data = temporary_core_data;
    temperature_data_.summary_status = level;
    temperature_data_.summary_message = temperature_dictionary_.at(level);
  }

  // Measure elapsed time since start time
  const auto t_end = std::chrono::high_resolution_clock::now();
  const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  temperature_data_.elapsed_ms = elapsed_ms;
}

void CPUMonitorBase::updateTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  int level = DiagStatus::OK;
  std::string error_str = "";

  for (const auto & entry : temperature_data_.core_data) {
    if (entry.status == DiagStatus::OK) {
      stat.addf(entry.label, "%.1f DegC", entry.temperature);
    } else {
      level = entry.status;
      error_str = entry.error_key;
      stat.add(entry.error_key, entry.error_value);
    }
  }

  if (!error_str.empty()) {
    stat.summary(temperature_data_.summary_status, error_str);
  } else {
    stat.summary(level, temperature_dictionary_.at(level));
  }
  stat.addf("execution time", "%f ms", temperature_data_.elapsed_ms);
}

void CPUMonitorBase::checkUsage()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<CpuUsageStatistics::CoreUsageInfo> core_usage_info;  // TODO(masakinishikawa): Allocated on heap.
  cpu_usage_statistics_.collect_cpu_statistics(core_usage_info);  // May take a while.

  std::lock_guard<std::mutex> lock(mutex_);
  usage_data_.clear();

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  for (const auto & usage : core_usage_info) {
    UsageData::CpuUsage cpu_usage;
    cpu_usage.label = usage.name;
    cpu_usage.usr = usage.user_percent;
    cpu_usage.nice = usage.nice_percent;
    cpu_usage.sys = usage.system_percent;
    cpu_usage.iowait = usage.iowait_percent;
    cpu_usage.idle = usage.idle_percent;
    cpu_usage.total = usage.total_usage_percent;
    // Some of the members of CpuUsageStatistics::CoreUsageInfo are not used.

    // CpuUsageToLevel() converts usage to warning and error levels.
    // It also counts occurence of warning and error in addition to conversion.
    float total_usage = usage.total_usage_percent * 1e-2;
    bool get_cpu_name = false;  // TODO(masakinishikawa): Check if cpu_name is available.
    if (get_cpu_name) {
      level = CpuUsageToLevel(usage.name, total_usage);
    } else {
      level = CpuUsageToLevel(std::string("err"), total_usage);
    }
    cpu_usage.status = level;

    if (usage_average_ == true) {
      if (usage.name == "all") {
        whole_level = level;
      }
    } else {
      whole_level = std::max(whole_level, level);
    }
    usage_data_.core_data.push_back(cpu_usage);
  }
  usage_data_.summary_status = whole_level;
  usage_data_.summary_message = load_dictionary_.at(whole_level);

  // Measure elapsed time since start time for reporting.
  const auto t_end = std::chrono::high_resolution_clock::now();
  const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  usage_data_.elapsed_ms = elapsed_ms;
}

void CPUMonitorBase::updateUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  tier4_external_api_msgs::msg::CpuUsage cpu_usage;
  using CpuStatus = tier4_external_api_msgs::msg::CpuStatus;

  if (usage_data_.summary_status != DiagStatus::OK) {
    stat.summary(usage_data_.summary_status, usage_data_.summary_message);
    stat.add(usage_data_.error_key, usage_data_.error_value);
    cpu_usage.all.status = CpuStatus::STALE;
    cpu_usage.cpus.clear();
    publishCpuUsage(cpu_usage);
    return;
  }

  for (const auto & usage : usage_data_.core_data) {
    CpuStatus cpu_status;
    cpu_status.status = usage.status;
    cpu_status.total = usage.total;
    cpu_status.usr = usage.usr;
    cpu_status.nice = usage.nice;
    cpu_status.sys = usage.sys;
    cpu_status.idle = usage.idle;
    if (usage.label == "all") {
      cpu_usage.all = cpu_status;
    } else {
      cpu_usage.cpus.push_back(cpu_status);
    }

    stat.add(fmt::format("CPU {}: status", usage.label), usage.status);
    stat.addf(fmt::format("CPU {}: total", usage.label), "%.2f%%", usage.total);
    stat.addf(fmt::format("CPU {}: usr", usage.label), "%.2f%%", usage.usr);
    stat.addf(fmt::format("CPU {}: nice", usage.label), "%.2f%%", usage.nice);
    stat.addf(fmt::format("CPU {}: sys", usage.label), "%.2f%%", usage.sys);
    stat.addf(fmt::format("CPU {}: idle", usage.label), "%.2f%%", usage.idle);
  }

  stat.summary(usage_data_.summary_status, usage_data_.summary_message);

  // Publish msg
  publishCpuUsage(cpu_usage);

  stat.addf("execution time", "%f ms", usage_data_.elapsed_ms);
}

int CPUMonitorBase::CpuUsageToLevel(const std::string & cpu_name, float usage)
{
  // cpu name to counter index
  int idx;
  try {
    int num = std::stoi(cpu_name);
    if (num > num_cores_ || num < 0) {
      num = num_cores_;
    }
    idx = num + 1;
  } catch (std::exception &) {
    if (cpu_name == std::string("all")) {  // mpstat output "all"
      idx = 0;
    } else {
      idx = num_cores_ + 1;
    }
  }

  // convert CPU usage to level
  int level = DiagStatus::OK;
  if (usage >= usage_warn_) {
    if (usage_warn_check_count_[idx] < usage_warn_count_) {
      usage_warn_check_count_[idx]++;
    }
    if (usage_warn_check_count_[idx] >= usage_warn_count_) {
      level = DiagStatus::WARN;
    }
  } else {
    usage_warn_check_count_[idx] = 0;
  }
  if (usage >= usage_error_) {
    if (usage_error_check_count_[idx] < usage_error_count_) {
      usage_error_check_count_[idx]++;
    }
    if (usage_error_check_count_[idx] >= usage_error_count_) {
      level = DiagStatus::ERROR;
    }
  } else {
    usage_error_check_count_[idx] = 0;
  }

  return level;
}

void CPUMonitorBase::checkLoad()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  double load_average[3];

  std::ifstream ifs("/proc/loadavg", std::ios::in);

  if (!ifs) {
    std::lock_guard<std::mutex> lock(mutex_);
    load_data_.clear();
    load_data_.summary_status = DiagStatus::ERROR;
    load_data_.summary_message = "uptime error";
    load_data_.elapsed_ms = 0.0f;
    load_data_.error_key = "uptime";
    load_data_.error_value = strerror(errno);
    return;
  }

  std::string line;

  if (!std::getline(ifs, line)) {
    std::lock_guard<std::mutex> lock(mutex_);
    load_data_.clear();
    load_data_.summary_status = DiagStatus::ERROR;
    load_data_.summary_message = "uptime error";
    load_data_.elapsed_ms = 0.0f;
    load_data_.error_key = "uptime";
    load_data_.error_value = "format error";
    return;
  }

  if (
    sscanf(line.c_str(), "%lf %lf %lf", &load_average[0], &load_average[1], &load_average[2]) !=
    3) {
    std::lock_guard<std::mutex> lock(mutex_);
    load_data_.clear();
    load_data_.summary_status = DiagStatus::ERROR;
    load_data_.summary_message = "uptime error";
    load_data_.elapsed_ms = 0.0f;
    load_data_.error_key = "uptime";
    load_data_.error_value = "format error";
    return;
  }

  load_average[0] /= num_cores_;
  load_average[1] /= num_cores_;
  load_average[2] /= num_cores_;

  std::lock_guard<std::mutex> lock(mutex_);
  load_data_.clear();
  load_data_.summary_status = DiagStatus::OK;
  load_data_.summary_message = "OK";
  load_data_.load_average[0] = load_average[0] * 1e2;
  load_data_.load_average[1] = load_average[1] * 1e2;
  load_data_.load_average[2] = load_average[2] * 1e2;

  // Measure elapsed time since start time for reporting.
  const auto t_end = std::chrono::high_resolution_clock::now();
  const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  load_data_.elapsed_ms = elapsed_ms;
}

void CPUMonitorBase::updateLoad(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (load_data_.summary_status != DiagStatus::OK) {
    stat.summary(load_data_.summary_status, load_data_.summary_message);
    stat.add(load_data_.error_key, load_data_.error_value);
    return;
  }

  stat.summary(load_data_.summary_status, load_data_.summary_message);
  stat.addf("1min", "%.2f%%", load_data_.load_average[0]);
  stat.addf("5min", "%.2f%%", load_data_.load_average[1]);
  stat.addf("15min", "%.2f%%", load_data_.load_average[2]);

  stat.addf("execution time", "%f ms", load_data_.elapsed_ms);
}

void CPUMonitorBase::checkThermalThrottling(
  [[maybe_unused]] diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::checkThermalThrottling not implemented.");
}

void CPUMonitorBase::checkFrequency()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  if (frequencies_.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    frequency_data_.clear();
    frequency_data_.summary_status = DiagStatus::ERROR;
    frequency_data_.summary_message = "frequency files not found";
    frequency_data_.elapsed_ms = 0.0f;
    return;
  }

  std::vector<FrequencyData::CoreFrequency> temporary_core_data{};

  for (const auto & entry : frequencies_) {
    // Read scaling_cur_freq file
    const fs::path path(entry.path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        temporary_core_data.emplace_back(
          FrequencyData::CoreFrequency{entry.index_, DiagStatus::OK, std::stoi(line)});
      }
    }
    ifs.close();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  frequency_data_.clear();
  frequency_data_.core_data = temporary_core_data;
  frequency_data_.summary_status = DiagStatus::OK;
  frequency_data_.summary_message = "OK";

  // Measure elapsed time since start time for reporting.
  const auto t_end = std::chrono::high_resolution_clock::now();
  const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  frequency_data_.elapsed_ms = elapsed_ms;
}

void CPUMonitorBase::updateFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (frequency_data_.summary_status != DiagStatus::OK) {
    stat.summary(frequency_data_.summary_status, frequency_data_.summary_message);
    return;
  }

  for (const auto & entry : frequency_data_.core_data) {
    stat.addf(fmt::format("CPU {}: clock", entry.index), "%d MHz", (entry.frequency_khz / 1000));
  }

  stat.summary(frequency_data_.summary_status, frequency_data_.summary_message);
  stat.addf("execution time", "%f ms", frequency_data_.elapsed_ms);
}

void CPUMonitorBase::getTemperatureFileNames()
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::getTemperatureFileNames not implemented.");
}

void CPUMonitorBase::getFrequencyFileNames()
{
  const fs::path root("/sys/devices/system/cpu");

  for (const fs::path & path :
       boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator())) {
    if (!fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const char * cpu_dir = path.generic_string().c_str();

    // /sys/devices/system/cpu[0-9] ?
    if (!std::regex_match(cpu_dir, match, std::regex(".*cpu(\\d+)"))) {
      continue;
    }

    // /sys/devices/system/cpu[0-9]/cpufreq/scaling_cur_freq
    CpuFrequencyInfo frequency;
    const fs::path frequency_path = path / "cpufreq/scaling_cur_freq";
    frequency.index_ = std::stoi(match[1].str());
    frequency.path_ = frequency_path.generic_string();
    frequencies_.push_back(frequency);
  }

  std::sort(
    frequencies_.begin(), frequencies_.end(),
    [](const CpuFrequencyInfo & c1, const CpuFrequencyInfo & c2) {
      return c1.index_ < c2.index_;
    });  // NOLINT
}

void CPUMonitorBase::publishCpuUsage(tier4_external_api_msgs::msg::CpuUsage usage)
{
  // Create timestamp
  const auto stamp = this->now();

  usage.stamp = stamp;
  pub_cpu_usage_->publish(usage);
}

void CPUMonitorBase::onTimer()
{
  checkTemperature();
  checkUsage();
  checkLoad();
  checkFrequency();
}
