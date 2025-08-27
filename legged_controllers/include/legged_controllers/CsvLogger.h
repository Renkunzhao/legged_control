#pragma once
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <Eigen/Dense>  // 头文件确保包含

namespace fs = std::filesystem;

class CsvLogger {
public:
  // 传入目录前缀，比如 "/tmp/legged_control_log"
  // 会自动创建 /tmp/legged_control_log_YYYYMMDD_HHMMSS/
  CsvLogger(const std::string& dirPrefix, const std::string& filename = "log.csv") {

    // 获取当前时间，格式化为字符串
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::ostringstream oss;
    oss << dirPrefix << "_"
        << std::put_time(&local_tm, "%Y%m%d_%H%M%S");

    logDir_ = oss.str();

    // 创建目录
    if (!fs::exists(logDir_)) {
      if (!fs::create_directories(logDir_)) {
        throw std::runtime_error("Failed to create log directory: " + logDir_);
      }
    }

    filename_ = logDir_ + "/" + filename;

    if (fs::exists(filename_)) {
      loadFieldOrderFromFile();
      file_.open(filename_, std::ios::app);
      headerWritten_ = true;
    } else {
      file_.open(filename_);
    }
  }

  ~CsvLogger() {
    if (file_.is_open()) file_.close();
  }


  // 单变量
  void log(const std::string& name, double value) {
    buffer_[name] = value;
    addField(name);
  }

  // 向量变量：自动展开成 name0, name1, ...
  void log(const std::string& name, const std::vector<double>& values) {
    for (size_t i = 0; i < values.size(); ++i) {
      std::string fullName = name + std::to_string(i);
      buffer_[fullName] = values[i];
      addField(fullName);
    }
  }

    // Eigen 向量重载
    void log(const std::string& name, const Eigen::VectorXd& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        std::string fullName = name + std::to_string(i);
        buffer_[fullName] = vec[i];
        addField(fullName);
    }
    }

  void flush() {
    // ① 添加此行：在写入前检查缓冲区是否为空
    // 如果缓冲区为空，说明本轮没有调用log方法，不需要写入数据
    if (buffer_.empty()) {
        return;
    }

    // ② 以下为原始逻辑，只有在缓冲区有数据时才执行

    if (!headerWritten_ || headerDirty_) {
      writeHeader();
      headerWritten_ = true;
      headerDirty_ = false;
    }

    for (size_t i = 0; i < fieldOrder_.size(); ++i) {
      const auto& key = fieldOrder_[i];
      if (buffer_.count(key)) {
        file_ << std::setprecision(10) << buffer_[key];
      } else {
        file_ << 0.0;  // 未写入字段补0
      }
      if (i + 1 < fieldOrder_.size()) file_ << ",";
    }
    file_ << "\n";
    buffer_.clear();
  }

private:
  std::string logDir_;
  std::string filename_;
  std::ofstream file_;
  std::map<std::string, double> buffer_;
  std::vector<std::string> fieldOrder_;
  bool headerWritten_ = false;
  bool headerDirty_ = false;

  void addField(const std::string& name) {
    if (std::find(fieldOrder_.begin(), fieldOrder_.end(), name) == fieldOrder_.end()) {
      fieldOrder_.push_back(name);
      headerDirty_ = true;
    }
  }

  void writeHeader() {
    // Rewind file and rewrite header
    file_.close();

    // 读取旧内容（不包含旧 header）
    std::ifstream oldFile(filename_);
    std::vector<std::string> oldLines;
    std::string line;
    bool skipFirst = true;
    while (std::getline(oldFile, line)) {
      if (skipFirst) { skipFirst = false; continue; } // skip header
      oldLines.push_back(line);
    }
    oldFile.close();

    // 重新写入
    file_.open(filename_, std::ios::trunc);
    for (size_t i = 0; i < fieldOrder_.size(); ++i) {
      file_ << fieldOrder_[i];
      if (i + 1 < fieldOrder_.size()) file_ << ",";
    }
    file_ << "\n";

    // 写入旧数据（默认补0）
    for (const auto& oldLine : oldLines) {
      std::vector<std::string> cols = split(oldLine, ',');
      size_t N = cols.size();
      for (size_t i = 0; i < fieldOrder_.size(); ++i) {
        if (i < N) {
          file_ << cols[i];
        } else {
          file_ << "0.0";
        }
        if (i + 1 < fieldOrder_.size()) file_ << ",";
      }
      file_ << "\n";
    }

    file_.flush();
  }

  void loadFieldOrderFromFile() {
    std::ifstream in(filename_);
    std::string line;
    if (std::getline(in, line)) {
      fieldOrder_ = split(line, ',');
    }
    in.close();
  }

  std::vector<std::string> split(const std::string& line, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, delim)) {
      tokens.push_back(item);
    }
    return tokens;
  }
};
