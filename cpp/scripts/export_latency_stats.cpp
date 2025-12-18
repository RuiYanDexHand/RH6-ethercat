// 延迟统计导出工具
// 用于将延迟统计数据导出为CSV格式，便于第三方工具分析
// 可以作为辅助工具集成到主程序中

#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <string>

// 延迟统计结构（与 ethercat_bridge.hpp 中的 LatencyStats 对应）
struct LatencyStatsCSV {
    int64_t cycle_min_us;
    int64_t cycle_max_us;
    int64_t cycle_avg_us;
    int64_t cycle_jitter_max_us;
    int64_t cycle_jitter_avg_us;
    
    int64_t ecat_receive_min_us;
    int64_t ecat_receive_max_us;
    int64_t ecat_receive_avg_us;
    
    int64_t ecat_process_min_us;
    int64_t ecat_process_max_us;
    int64_t ecat_process_avg_us;
    
    int64_t ecat_send_min_us;
    int64_t ecat_send_max_us;
    int64_t ecat_send_avg_us;
    
    int64_t total_cycle_min_us;
    int64_t total_cycle_max_us;
    int64_t total_cycle_avg_us;
    
    int64_t samples;
};

void export_to_csv(const LatencyStatsCSV& stats, const std::string& filename) {
    std::ofstream file(filename, std::ios::app);
    
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }
    
    // 如果是新文件，写入CSV头部
    file.seekp(0, std::ios::end);
    bool is_new_file = (file.tellp() == 0);
    
    if (is_new_file) {
        file << "timestamp,"
             << "cycle_min_us,cycle_max_us,cycle_avg_us,cycle_jitter_max_us,cycle_jitter_avg_us,"
             << "ecat_receive_min_us,ecat_receive_max_us,ecat_receive_avg_us,"
             << "ecat_process_min_us,ecat_process_max_us,ecat_process_avg_us,"
             << "ecat_send_min_us,ecat_send_max_us,ecat_send_avg_us,"
             << "total_cycle_min_us,total_cycle_max_us,total_cycle_avg_us,"
             << "samples"
             << std::endl;
    }
    
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::tm* timeinfo = std::localtime(&time_t);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    
    // 写入数据
    file << std::fixed << std::setprecision(3)
         << buffer << "." << std::setfill('0') << std::setw(3) << ms.count() << ","
         << stats.cycle_min_us << ","
         << stats.cycle_max_us << ","
         << stats.cycle_avg_us << ","
         << stats.cycle_jitter_max_us << ","
         << stats.cycle_jitter_avg_us << ","
         << stats.ecat_receive_min_us << ","
         << stats.ecat_receive_max_us << ","
         << stats.ecat_receive_avg_us << ","
         << stats.ecat_process_min_us << ","
         << stats.ecat_process_max_us << ","
         << stats.ecat_process_avg_us << ","
         << stats.ecat_send_min_us << ","
         << stats.ecat_send_max_us << ","
         << stats.ecat_send_avg_us << ","
         << stats.total_cycle_min_us << ","
         << stats.total_cycle_max_us << ","
         << stats.total_cycle_avg_us << ","
         << stats.samples
         << std::endl;
    
    file.close();
    std::cout << "[INFO] 延迟统计已导出到: " << filename << std::endl;
}

// 示例用法
int main() {
    LatencyStatsCSV stats = {
        .cycle_min_us = 998,
        .cycle_max_us = 1005,
        .cycle_avg_us = 1000,
        .cycle_jitter_max_us = 5,
        .cycle_jitter_avg_us = 2,
        .ecat_receive_min_us = 45,
        .ecat_receive_max_us = 60,
        .ecat_receive_avg_us = 50,
        .ecat_process_min_us = 25,
        .ecat_process_max_us = 35,
        .ecat_process_avg_us = 30,
        .ecat_send_min_us = 18,
        .ecat_send_max_us = 25,
        .ecat_send_avg_us = 20,
        .total_cycle_min_us = 88,
        .total_cycle_max_us = 120,
        .total_cycle_avg_us = 100,
        .samples = 1000
    };
    
    export_to_csv(stats, "latency_stats.csv");
    return 0;
}

