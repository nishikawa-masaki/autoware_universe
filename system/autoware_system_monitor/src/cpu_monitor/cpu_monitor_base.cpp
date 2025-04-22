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

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <chrono>  // for 1s
#include <cstdio>
#include <regex>
#include <string>
#include <utility>

namespace bp = boost::process;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

CPUMonitorBase::CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  hostname_(),
  num_cores_(0),
  temperatures_(),
  frequencies_(),
  usage_warn_(declare_parameter<float>("usage_warn", 0.96,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description("Threshold for CPU usage warning. Cannot be changed after initialization."))),
  usage_error_(declare_parameter<float>("usage_error", 0.96,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description("Threshold for CPU usage error. Cannot be changed after initialization."))),
  usage_warn_count_(declare_parameter<int>("usage_warn_count", 1,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description("Consecutive count threshold for CPU usage warning. Cannot be changed after initialization."))),
  usage_error_count_(declare_parameter<int>("usage_error_count", 2,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description("Consecutive count threshold for CPU usage error. Cannot be changed after initialization."))),
  usage_average_(declare_parameter<bool>("usage_avg", true,
    rcl_interfaces::msg::ParameterDescriptor().set__read_only(true).set__description("Use average CPU usage across all processors. Cannot be changed after initialization.")))
{
  gethostname(hostname_, sizeof(hostname_));
  num_cores_ = boost::thread::hardware_concurrency();
  usage_warn_check_count_.resize(num_cores_ + 2);   // 2 = all + dummy
  usage_error_check_count_.resize(num_cores_ + 2);  // 2 = all + dummy

  updater_.setHardwareID(hostname_);
  updater_.add("CPU Temperature", this, &CPUMonitorBase::updateTemperature);
#if 1
  updater_.add("CPU Usage", this, &CPUMonitorBase::checkUsage);
  updater_.add("CPU Load Average", this, &CPUMonitorBase::checkLoad);
  updater_.add("CPU Thermal Throttling", this, &CPUMonitorBase::checkThermalThrottling);
  updater_.add("CPU Frequency", this, &CPUMonitorBase::checkFrequency);
#else  // 0
  updater_.add("CPU Usage", this, &CPUMonitorBase::updateUsage);
  updater_.add("CPU Load Average", this, &CPUMonitorBase::updateLoad);
  updater_.add("CPU Thermal Throttling", this, &CPUMonitorBase::updateThermalThrottling);
  updater_.add("CPU Frequency", this, &CPUMonitorBase::updateFrequency);
#endif  // 0
  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_cpu_usage_ =
    this->create_publisher<tier4_external_api_msgs::msg::CpuUsage>("~/cpu_usage", durable_qos);

  using namespace std::chrono_literals;
  // Start timer to execute collect cpu statistics
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&CPUMonitorBase::onTimer, this), timer_callback_group_);
}

void CPUMonitorBase::update()
{
  updater_.force_update();
}

void CPUMonitorBase::checkTemperature()
{
  // Remember start time to measure elapsed time
  const auto t_start = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  temperature_data_.core_data.clear();  // Data content is cleared, but memory is not freed

  if (temperatures_.empty()) {
    temperature_data_.summary_status = DiagStatus::ERROR;
    temperature_data_.summary_message = "temperature files not found";
    temperature_data_.core_data.clear();  // Data content is cleared, but memory is not freed
    temperature_data_.elapsed_ms = 0.0f;
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = temperatures_.begin(); itr != temperatures_.end(); ++itr) {
    // Read temperature file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      error_str = "file open error";
      temperature_data_.core_data.emplace_back(
        TemperatureData::CoreTemperature{itr->label_, DiagStatus::ERROR, 0.0f, error_str, itr->path_});
      continue;
    }

    float temperature;
    ifs >> temperature;
    ifs.close();
    temperature /= 1000;
    temperature_data_.core_data.emplace_back(
      TemperatureData::CoreTemperature{itr->label_, DiagStatus::OK, temperature, "", ""});
  }

  if (!error_str.empty()) {
    temperature_data_.summary_status = DiagStatus::ERROR;
    temperature_data_.summary_message = error_str;
  } else {
    temperature_data_.summary_status = level;
    temperature_data_.summary_message = temperature_dictionary_.at(level);
  }

  // Measure elapsed time since start time and report
  const auto t_end = std::chrono::high_resolution_clock::now();
  const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  temperature_data_.elapsed_ms = elapsed_ms;
}

void CPUMonitorBase::updateTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  int level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = temperature_data_.core_data.begin(); itr != temperature_data_.core_data.end(); ++itr) {
    if (itr->status == DiagStatus::OK) {
      stat.addf(itr->label, "%.1f DegC", itr->temperature);
    } else {
      level = itr->status;
      error_str = itr->error_key;
      stat.add(itr->error_key, itr->error_value);
    }
  }

  if (!error_str.empty()) {
    stat.summary(temperature_data_.summary_status, error_str);
  } else {
    stat.summary(level, temperature_dictionary_.at(level));
  }
  stat.addf("execution time", "%f ms", temperature_data_.elapsed_ms);
}

void CPUMonitorBase::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  tier4_external_api_msgs::msg::CpuUsage cpu_usage;
  using CpuStatus = tier4_external_api_msgs::msg::CpuStatus;

#if 0
  if (!mpstat_exists_) {
    stat.summary(DiagStatus::ERROR, "mpstat error");
    stat.add(
      "mpstat", "Command 'mpstat' not found, but can be installed with: sudo apt install sysstat");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }
#endif  // 0

  // Get CPU Usage

  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("mpstat -P ALL 1 1 -o JSON", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "mpstat error");
    stat.add("mpstat", os.str().c_str());
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  pt::ptree pt;
  try {
    // Analyze JSON output
    read_json(is_out, pt);

    for (const pt::ptree::value_type & child1 : pt.get_child("sysstat.hosts")) {
      const pt::ptree & hosts = child1.second;

      for (const pt::ptree::value_type & child2 : hosts.get_child("statistics")) {
        const pt::ptree & statistics = child2.second;

        for (const pt::ptree::value_type & child3 : statistics.get_child("cpu-load")) {
          const pt::ptree & cpu_load = child3.second;
          bool get_cpu_name = false;

          CpuStatus cpu_status;

          std::string cpu_name;
          float usr{0.0f};
          float nice{0.0f};
          float sys{0.0f};
          float iowait{0.0f};
          float idle{0.0f};
          float usage{0.0f};
          float total{0.0f};

          if (boost::optional<std::string> v = cpu_load.get_optional<std::string>("cpu")) {
            cpu_name = v.get();
            get_cpu_name = true;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("usr")) {
            usr = v.get();
            cpu_status.usr = usr;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("nice")) {
            nice = v.get();
            cpu_status.nice = nice;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("sys")) {
            sys = v.get();
            cpu_status.sys = sys;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("idle")) {
            idle = v.get();
            cpu_status.idle = idle;
          }

          total = 100.0f - iowait - idle;
          usage = total * 1e-2;
          if (get_cpu_name) {
            level = CpuUsageToLevel(cpu_name, usage);
          } else {
            level = CpuUsageToLevel(std::string("err"), usage);
          }

          cpu_status.total = total;
          cpu_status.status = level;

          stat.add(fmt::format("CPU {}: status", cpu_name), load_dictionary_.at(level));
          stat.addf(fmt::format("CPU {}: total", cpu_name), "%.2f%%", total);
          stat.addf(fmt::format("CPU {}: usr", cpu_name), "%.2f%%", usr);
          stat.addf(fmt::format("CPU {}: nice", cpu_name), "%.2f%%", nice);
          stat.addf(fmt::format("CPU {}: sys", cpu_name), "%.2f%%", sys);
          stat.addf(fmt::format("CPU {}: idle", cpu_name), "%.2f%%", idle);

          if (usage_average_ == true) {
            if (cpu_name == "all") {
              whole_level = level;
            }
          } else {
            whole_level = std::max(whole_level, level);
          }

          if (cpu_name == "all") {
            cpu_usage.all = cpu_status;
          } else {
            cpu_usage.cpus.push_back(cpu_status);
          }
        }
      }
    }
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "mpstat exception");
    stat.add("mpstat", e.what());
    std::fill(usage_warn_check_count_.begin(), usage_warn_check_count_.end(), 0);
    std::fill(usage_error_check_count_.begin(), usage_error_check_count_.end(), 0);
    cpu_usage.all.status = CpuStatus::STALE;
    cpu_usage.cpus.clear();
    publishCpuUsage(cpu_usage);
    return;
  }

  stat.summary(whole_level, load_dictionary_.at(whole_level));

  // Publish msg
  publishCpuUsage(cpu_usage);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
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

void CPUMonitorBase::checkLoad(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  double load_average[3];

  std::ifstream ifs("/proc/loadavg", std::ios::in);

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", strerror(errno));
    return;
  }

  std::string line;

  if (!std::getline(ifs, line)) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  if (sscanf(line.c_str(), "%lf %lf %lf", &load_average[0], &load_average[1], &load_average[2]) != 3) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  load_average[0] /= num_cores_;
  load_average[1] /= num_cores_;
  load_average[2] /= num_cores_;

  stat.summary(DiagStatus::OK, "OK");
  stat.addf("1min", "%.2f%%", load_average[0] * 1e2);
  stat.addf("5min", "%.2f%%", load_average[1] * 1e2);
  stat.addf("15min", "%.2f%%", load_average[2] * 1e2);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::checkThermalThrottling(
  [[maybe_unused]] diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::checkThermalThrottling not implemented.");
}

void CPUMonitorBase::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (frequencies_.empty()) {
    stat.summary(DiagStatus::ERROR, "frequency files not found");
    return;
  }

  for (auto itr = frequencies_.begin(); itr != frequencies_.end(); ++itr) {
    // Read scaling_cur_freq file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        stat.addf(fmt::format("CPU {}: clock", itr->index_), "%d MHz", std::stoi(line) / 1000);
      }
    }
    ifs.close();
  }

  stat.summary(DiagStatus::OK, "OK");

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
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

  std::sort(frequencies_.begin(), frequencies_.end(), [](const CpuFrequencyInfo & c1, const CpuFrequencyInfo & c2) {
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

void CPUMonitorBase::onTimer() {
  checkTemperature();
}