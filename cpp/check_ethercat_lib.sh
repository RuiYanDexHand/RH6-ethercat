#!/bin/bash

echo "=== EtherCAT 库检查脚本 ==="
echo

# 检查常见的 EtherCAT 库文件
echo "1. 检查常见的 EtherCAT 库文件："
echo "----------------------------------------"
for lib in libecrt.so libethercat.so ecrt; do
    echo -n "查找 $lib: "
    if find /usr -name "*$lib*" 2>/dev/null | head -1; then
        echo "✓ 找到"
    else
        echo "✗ 未找到"
    fi
done

echo
echo "2. 检查 IGH EtherCAT 相关文件："
echo "----------------------------------------"
if [ -d "/opt/ethercat" ]; then
    echo "✓ /opt/ethercat 目录存在"
    find /opt/ethercat -name "*.so*" 2>/dev/null | head -5
else
    echo "✗ /opt/ethercat 目录不存在"
fi

if [ -d "/usr/local/ethercat" ]; then
    echo "✓ /usr/local/ethercat 目录存在"
    find /usr/local/ethercat -name "*.so*" 2>/dev/null | head -5
else
    echo "✗ /usr/local/ethercat 目录不存在"
fi

echo
echo "3. 检查系统库路径："
echo "----------------------------------------"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "PKG_CONFIG_PATH: $PKG_CONFIG_PATH"

echo
echo "4. 检查 pkg-config 信息："
echo "----------------------------------------"
if pkg-config --exists libethercat 2>/dev/null; then
    echo "✓ libethercat pkg-config 信息存在"
    echo "  库路径: $(pkg-config --libs libethercat)"
    echo "  头文件路径: $(pkg-config --cflags libethercat)"
else
    echo "✗ libethercat pkg-config 信息不存在"
fi

echo
echo "5. 检查已安装的 EtherCAT 相关包："
echo "----------------------------------------"
if command -v dpkg >/dev/null 2>&1; then
    dpkg -l | grep -i ethercat
elif command -v rpm >/dev/null 2>&1; then
    rpm -qa | grep -i ethercat
else
    echo "无法检查已安装的包"
fi

echo
echo "6. 检查 EtherCAT 工具："
echo "----------------------------------------"
if command -v ethercat >/dev/null 2>&1; then
    echo "✓ ethercat 命令可用"
    ethercat --version 2>/dev/null || echo "无法获取版本信息"
else
    echo "✗ ethercat 命令不可用"
fi

echo
echo "=== 检查完成 ==="
