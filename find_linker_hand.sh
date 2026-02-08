#!/bin/bash

# ==========================================
# 配置区域
# 请在此处填写您的 sudo 密码 (如果没有密码或希望每次手动输入，请留空)
SUDO_PASS=""
# ==========================================

# 检查 can-utils 是否已安装
if ! command -v cansend &> /dev/null; then
    echo "错误: 未找到 cansend (can-utils)。"
    echo "请安装 can-utils: sudo apt update && sudo apt install can-utils"
    exit 1
fi

if ! command -v candump &> /dev/null; then
    echo "错误: 未找到 candump (can-utils)。"
    echo "请安装 can-utils: sudo apt update && sudo apt install can-utils"
    exit 1
fi

echo "正在扫描可用的 CAN 接口..."

# 获取 CAN 接口列表
mapfile -t interfaces < <(ip -br link show type can | awk '{print $1}')

if [ ${#interfaces[@]} -eq 0 ]; then
    echo "未找到任何 CAN 接口。"
    exit 0
fi

echo "发现 ${#interfaces[@]} 个接口: ${interfaces[*]}"
echo "正在向每个接口发送帧数据 [ID: 0xFF, Data: 0xC0]..."

# 目标帧
FRAME_ID="0FF"
FRAME_DATA="C0"

# 定义一个帮助函数用于执行 sudo 命令
run_sudo() {
    if [ -z "$SUDO_PASS" ]; then
        sudo "$@"
    else
        echo "$SUDO_PASS" | sudo -S "$@"
    fi
}

for iface in "${interfaces[@]}"; do
    echo "==========================================="
    echo "正在处理接口: $iface"

    # 检查接口状态
    if ip link show "$iface" | grep -q "UP"; then
        echo "状态: 接口已激活"
    else
        echo "状态: 接口未激活，正在尝试激活..."
        # 尝试激活接口
        # 使用用户指定的完整命令格式
        if run_sudo /usr/sbin/ip link set "$iface" up type can bitrate 1000000; then
            echo "已成功激活接口 $iface"
            sleep 0.5 # 稍微等待接口稳定
        else
            echo "激活接口 $iface 失败。"
            echo "跳过后续操作。"
            continue
        fi
    fi

    echo "正在发送请求指令..."

    # 发送帧
    if ! cansend "$iface" "${FRAME_ID}#${FRAME_DATA}"; then
        echo "发送指令失败"
        continue
    fi

    echo "等待设备响应 (接收数据中)..."

    # 接收响应数据
    # 注意：接收所有ID的数据，因为返回帧ID可能不是 0xFF
    # 使用临时文件和后台任务确保不丢失数据
    
    # 创建临时文件
    TMP_FILE=$(mktemp)
    
    # 后台启动 candump
    candump "$iface" > "$TMP_FILE" 2>&1 &
    CANDUMP_PID=$!
    
    # 稍微等待让 candump 初始化
    sleep 0.2
    
    # 发送帧
    cansend "$iface" "${FRAME_ID}#${FRAME_DATA}"
    
    # 等待设备回复
    sleep 1
    
    # 停止 candump
    kill $CANDUMP_PID 2>/dev/null
    wait $CANDUMP_PID 2>/dev/null
    
    # 读取数据
    RAW_DATA=$(cat "$TMP_FILE")
    rm -f "$TMP_FILE"

    echo "原始数据:"
    echo "$RAW_DATA"
    echo ""

    # 处理解析结果
    # 数据说明:
    # BYTE0: C0 (指令码)
    # BYTE1: 序号 (01, 02...)
    # BYTE2-7: 有效数据 (ASCII)
    
    # 定义扫描结果变量
    SCAN_RESULT=""
    
    # 提取并拼接
    if command -v xxd &> /dev/null; then
        # 提取数据并排序
        SORTED_DATA=$(echo "$RAW_DATA" | awk '{
            # 重组行
            data = ""
            for (i=4; i<=NF; i++) {
                data = data $i
            }
            gsub(/[[:space:]]/, "", data)
            if (length(data) < 5) next
            
            seq = substr(data, 3, 2)
            raw_bytes = substr(data, 5)
            
            if (length(raw_bytes) == 0) next
            
            # 输出序号和原始十六进制数据
            print seq, raw_bytes
        }' | sort -k1,1)
        
        # 检查是否有数据
        if [ -z "$SORTED_DATA" ]; then
            echo "未能解析到设备串码。"
            SCAN_RESULT="RETRY"
        fi
        
        if [ -z "$SCAN_RESULT" ]; then
            # 拼接所有十六进制字节
            HEX_STR=$(echo "$SORTED_DATA" | awk '{printf "%s", $2}')
            
            # 检查 HEX_STR 是否为空
            if [ -z "$HEX_STR" ]; then
                echo "未能解析到设备串码 (空数据)。"
                SCAN_RESULT="RETRY"
            fi
        fi
        
        if [ -z "$SCAN_RESULT" ]; then
            # 转换为 ASCII
            # 使用 bash 内联替换避免复杂的引号嵌套问题
            SERIAL_NUM=$(echo "$HEX_STR" | xxd -r -p | tr -d '\0')
        fi
        
    else
        # Fallback: 简单拼接，不转换 ASCII
        SERIAL_NUM=$(echo "$RAW_DATA" | awk '{
            data = ""
            for (i=4; i<=NF; i++) {
                data = data $i
            }
            gsub(/[[:space:]]/, "", data)
            if (length(data) < 5) next
            seq = substr(data, 3, 2)
            chars = substr(data, 5, 12)
            print seq, chars
        }' | sort -k1,1 | awk '{printf "%s", $2}')
        
        if [ -z "$SERIAL_NUM" ]; then
            SCAN_RESULT="RETRY"
        fi
    fi

    # 如果首次扫描未能解析到设备串码，发送0xFF帧ID，数据帧为0x01进行重试
    if [ "$SCAN_RESULT" = "RETRY" ]; then
        echo "首次扫描未能解析到设备串码，正在发送重试指令 [ID: 0xFF, Data: 0x01]..."
        
        # 创建临时文件用于接收重试响应
        TMP_FILE_RETRY=$(mktemp)
        
        # 后台启动 candump 接收重试响应
        candump "$iface" > "$TMP_FILE_RETRY" 2>&1 &
        CANDUMP_PID_RETRY=$!
        
        # 稍微等待让 candump 初始化
        sleep 0.2
        
        # 发送 0xFF 帧 ID，数据帧为 0x01
        cansend "$iface" "0FF#01"
        
        # 等待设备回复
        sleep 1
        
        # 停止 candump
        kill $CANDUMP_PID_RETRY 2>/dev/null
        wait $CANDUMP_PID_RETRY 2>/dev/null
        
        # 读取重试响应数据
        RETRY_RAW_DATA=$(cat "$TMP_FILE_RETRY")
        rm -f "$TMP_FILE_RETRY"
        
        echo "重试响应原始数据:"
        echo "$RETRY_RAW_DATA"
        echo ""
        
        # 提取响应帧 ID 并判断左右手
        # 格式示例: can1  1234   [8]  01 02 03 04 05 06 07 08
        RESPONSE_FRAME_ID=$(echo "$RETRY_RAW_DATA" | awk '{
            # 查找匹配 CAN ID 的行
            for (i=1; i<=NF; i++) {
                if ($i ~ /^[0-9A-Fa-f]+$/) {
                    # 提取帧 ID (去掉 0x 前缀并转为大写)
                    frame_id = toupper($i)
                    gsub(/^0X/, "", frame_id)
                    print frame_id
                    exit
                }
            }
        }')
        
        # 判断响应帧 ID
        if [ "$RESPONSE_FRAME_ID" = "28" ]; then
            echo "扫描到左手在 $iface 端口"
        elif [ "$RESPONSE_FRAME_ID" = "27" ]; then
            echo "扫描到右手在 $iface 端口"
        else
            echo "重试扫描未能识别设备类型 (响应帧ID: $RESPONSE_FRAME_ID)"
        fi
    elif [ -n "$SERIAL_NUM" ]; then
        echo "设备出厂编码 (手背串码): $SERIAL_NUM"
    else
        echo "未能解析到设备串码。"
    fi

    echo ""
done

echo "CAN端口扫描操作完成。"
