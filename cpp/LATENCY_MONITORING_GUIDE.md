# EtherCAT 系统延迟监控工具指南

本文档介绍如何使用第三方工具监控 EtherCAT 系统的延迟和性能。

## 📊 推荐工具分类

### 1. 实时性能测试工具（强烈推荐）

#### **cyclictest** - RT-Linux 延迟测试
**适用场景**: 测量系统实时延迟、调度延迟

**安装**:
```bash
sudo apt-get install rt-tests
# 或从源码编译
git clone https://git.kernel.org/pub/scm/utils/rt-tests/rt-tests.git
cd rt-tests && make && sudo make install
```

**常用命令**:
```bash
# 基本测试（单线程，优先级99，1ms周期，10000次）
sudo cyclictest -t1 -p 99 -i 1000 -l 10000

# 详细输出（显示最小/最大/平均延迟）
sudo cyclictest -t1 -p 99 -i 1000 -l 10000 -v

# 持续测试60秒，显示统计
sudo cyclictest -t1 -p 99 -i 1000 -l 1000000 -D 60 -q

# 高精度测试（使用nanosleep）
sudo cyclictest -t1 -p 99 -n -i 1000 -l 10000

# 生成直方图数据（CSV格式）
sudo cyclictest -t1 -p 99 -i 1000 -l 100000 --histogram=100 > histogram.txt

# 多线程测试（模拟多核）
sudo cyclictest -t4 -p 99 -i 1000 -l 10000

# 测试特定CPU核心
sudo cyclictest -t1 -p 99 -i 1000 -l 10000 -a 0,1,2,3
```

**结果解读**:
```
T: 0 (  1234) P:99 I:1000 C: 10000 Min:      5 Act:   12 Avg:   15 Max:      45
```
- **Min**: 最小延迟（越小越好，理想 < 10 us）
- **Act**: 当前延迟
- **Avg**: 平均延迟（应接近周期时间）
- **Max**: 最大延迟（关键指标，应 < 100 us）

**可视化**:
```bash
# 生成直方图
sudo apt-get install gnuplot
rt-tests/cyclictest -t1 -p 99 -i 1000 -l 100000 --histogram=100 > hist.txt
rt-tests/histogram hist.txt > histogram.png
```

---

#### **rt-tests 套件其他工具**

**hackbench** - 进程间通信延迟
```bash
sudo hackbench -r 100 -g 10  # 100次运行，10组
```

**xeno-test** - Xenomai 实时性测试（如果使用 Xenomai）
```bash
sudo xeno-test --latency
```

---

### 2. Linux 性能分析工具

#### **perf** - 系统级性能分析
**适用场景**: CPU性能分析、函数调用分析

```bash
# 安装
sudo apt-get install linux-perf

# 监控进程性能
sudo perf stat -e cycles,instructions,cache-misses -p $(pidof hand_system_bridge)

# 记录性能数据（采样频率1000Hz）
sudo perf record -F 1000 -g -p $(pidof hand_system_bridge)

# 查看调用图
sudo perf report --call-graph

# 实时监控
sudo perf top -p $(pidof hand_system_bridge)

# 分析特定函数
sudo perf record -e cpu-clock -g -p $(pidof hand_system_bridge)
sudo perf annotate ecrt_master_receive
```

#### **ftrace** - 内核函数跟踪
**适用场景**: 内核级延迟分析

```bash
# 启用跟踪
sudo mount -t debugfs nodev /sys/kernel/debug
echo 1 > /sys/kernel/debug/tracing/tracing_on

# 跟踪调度器
echo 1 > /sys/kernel/debug/tracing/events/sched/enable

# 查看结果
cat /sys/kernel/debug/tracing/trace

# 跟踪特定进程
echo $$ > /sys/kernel/debug/tracing/set_ftrace_pid
```

#### **eBPF/bcc-tools** - 高级跟踪
**适用场景**: 实时系统调用跟踪、函数延迟分析

```bash
# 安装
sudo apt-get install bpfcc-tools linux-headers-$(uname -r)

# 跟踪函数延迟
sudo funclatency-bpfcc ecrt_master_receive

# 跟踪系统调用
sudo syscalllat-bpfcc 'sys_nanosleep'

# 跟踪内核函数
sudo trace-bpfcc 'c:ecrt_master_receive() "%d us", retval'
```

---

### 3. EtherCAT 专用工具

#### **ethercat** - EtherCAT 诊断工具
**适用场景**: EtherCAT 主站和从站状态监控

```bash
# 查看主站状态
ethercat master

# 查看从站信息
ethercat slaves -v

# 查看应用时间
ethercat master --app-time

# 持续监控（每100ms刷新）
watch -n 0.1 'ethercat master'

# 查看PDO映射
ethercat pdos

# 查看从站对象字典
ethercat objects -p 0
```

---

### 4. 系统监控工具

#### **htop/atop** - 系统资源监控
```bash
sudo apt-get install htop atop

# htop - 实时查看CPU、内存使用
htop

# atop - 历史记录
atop -w /tmp/atop.log  # 记录到文件
```

#### **iostat** - I/O 性能监控
```bash
sudo apt-get install sysstat

# 监控I/O延迟
iostat -x 1
```

#### **sar** - 系统活动报告
```bash
# 安装
sudo apt-get install sysstat

# 监控CPU使用率
sar -u 1

# 监控内存
sar -r 1

# 保存数据
sar -u 1 100 > cpu_usage.txt
```

---

### 5. 可视化工具

#### **Grafana + Prometheus** - 时序数据可视化
**适用场景**: 长期监控、趋势分析

**安装**:
```bash
# Prometheus
wget https://github.com/prometheus/prometheus/releases/download/v2.x.x/prometheus-2.x.x.linux-arm64.tar.gz
tar xvfz prometheus-*.tar.gz

# Grafana
sudo apt-get install -y software-properties-common
sudo add-apt-repository "deb https://packages.grafana.com/oss/deb stable main"
sudo apt-get update
sudo apt-get install grafana
```

**配置**:
- Prometheus 配置: 添加你的应用作为监控目标
- Grafana: 导入仪表板，配置数据源

#### **gnuplot** - 绘制延迟曲线
```bash
sudo apt-get install gnuplot

# 从CSV文件绘制
gnuplot << EOF
set datafile separator ','
set xlabel 'Time'
set ylabel 'Latency (us)'
set title 'EtherCAT Cycle Latency'
plot 'latency_stats.csv' using 1:3 with lines title 'Average'
EOF
```

---

## 🔧 集成到你的程序

### 方法1: CSV导出（推荐）

在 `ethercat_bridge.cpp` 中添加CSV导出功能：

```cpp
// 在 print_latency_stats() 函数中添加
void ethercat_bridge::export_latency_stats_csv(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(latency_stats_mutex_);
    
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) return;
    
    // CSV格式输出
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    file << time_t << ","
         << latency_stats_.cycle_avg_us << ","
         << latency_stats_.cycle_max_us << ","
         << latency_stats_.ecat_receive_avg_us << ","
         << latency_stats_.ecat_process_avg_us << ","
         << latency_stats_.total_cycle_avg_us
         << std::endl;
}
```

### 方法2: ROS2 话题发布

创建一个延迟统计话题发布器：
```cpp
// 发布延迟统计数据到ROS2话题
void ethercat_bridge::publish_latency_stats() {
    auto stats = get_latency_stats();
    
    // 创建消息并发布
    // latency_pub_->publish(stats);
}
```

---

## 📋 实际使用建议

### 1. **开发阶段**
- 使用 `cyclictest` 验证系统实时性
- 使用 `perf` 分析性能瓶颈
- 使用 `ethercat master` 监控EtherCAT状态

### 2. **调试阶段**
- 使用 `ftrace` 跟踪内核函数
- 使用 `bcc-tools` 跟踪特定函数
- 导出CSV数据用 `gnuplot` 可视化

### 3. **生产环境**
- 集成CSV导出功能
- 使用 `Grafana` 长期监控
- 设置告警阈值

---

## ⚠️ 注意事项

1. **权限**: 大多数工具需要 `sudo` 权限
2. **性能影响**: 监控工具本身会消耗系统资源，建议在测试环境使用
3. **实时内核**: 某些工具（如 cyclictest）在实时内核下效果更好
4. **采样频率**: 根据系统负载调整采样频率，避免过度采样

---

## 📚 参考资源

- **rt-tests**: https://git.kernel.org/pub/scm/utils/rt-tests/rt-tests.git
- **perf**: https://perf.wiki.kernel.org/
- **bcc-tools**: https://github.com/iovisor/bcc
- **EtherCAT**: https://www.etherlab.org/
- **Grafana**: https://grafana.com/docs/

---

## 🎯 快速开始

```bash
# 1. 测试系统实时性
sudo cyclictest -t1 -p 99 -i 1000 -l 10000

# 2. 监控你的程序
sudo perf top -p $(pidof hand_system_bridge)

# 3. 查看EtherCAT状态
ethercat master

# 4. 系统监控
htop
```

