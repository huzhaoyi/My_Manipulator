#!/bin/bash
#
# M5 Grasp 日志清理脚本
# 用于清理过期日志文件，释放磁盘空间
#
# 用法:
#   ./clean_logs.sh              # 使用默认配置清理
#   ./clean_logs.sh -d 7         # 删除7天前的日志
#   ./clean_logs.sh -s 100       # 删除超过100MB的日志目录时才清理
#   ./clean_logs.sh -p /var/log  # 指定日志目录
#   ./clean_logs.sh -a           # 删除所有日志
#   ./clean_logs.sh -n           # 干运行，只显示不删除
#

set -e

# ==================== 默认配置 ====================
LOG_DIR="/tmp/m5_grasp_logs"
KEEP_DAYS=7                    # 保留最近N天的日志
SIZE_THRESHOLD_MB=100          # 目录超过此大小才清理
DRY_RUN=false                  # 干运行模式
DELETE_ALL=false               # 删除所有日志

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ==================== 函数定义 ====================

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

show_help() {
    cat << EOF
M5 Grasp 日志清理脚本

用法: $0 [选项]

选项:
  -d, --days NUM       保留最近NUM天的日志 (默认: $KEEP_DAYS)
  -s, --size NUM       目录超过NUM MB才清理 (默认: $SIZE_THRESHOLD_MB)
  -p, --path PATH      日志目录路径 (默认: $LOG_DIR)
  -a, --all            删除所有日志文件
  -n, --dry-run        干运行，只显示要删除的文件，不实际删除
  -h, --help           显示此帮助信息

示例:
  $0                   # 使用默认配置清理
  $0 -d 3              # 只保留最近3天的日志
  $0 -a -n             # 干运行，显示所有将被删除的日志
  $0 -p /var/log/m5    # 清理指定目录的日志

EOF
}

get_dir_size_mb() {
    local dir=$1
    if [ -d "$dir" ]; then
        du -sm "$dir" 2>/dev/null | cut -f1
    else
        echo 0
    fi
}

count_files() {
    local dir=$1
    local pattern=$2
    find "$dir" -name "$pattern" -type f 2>/dev/null | wc -l
}

# ==================== 参数解析 ====================

while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--days)
            KEEP_DAYS="$2"
            shift 2
            ;;
        -s|--size)
            SIZE_THRESHOLD_MB="$2"
            shift 2
            ;;
        -p|--path)
            LOG_DIR="$2"
            shift 2
            ;;
        -a|--all)
            DELETE_ALL=true
            shift
            ;;
        -n|--dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# ==================== 主逻辑 ====================

echo "=========================================="
echo "       M5 Grasp 日志清理工具"
echo "=========================================="
echo ""

# 检查目录是否存在
if [ ! -d "$LOG_DIR" ]; then
    print_warn "日志目录不存在: $LOG_DIR"
    exit 0
fi

# 显示当前状态
CURRENT_SIZE=$(get_dir_size_mb "$LOG_DIR")
TOTAL_FILES=$(count_files "$LOG_DIR" "*.log*")
OLD_FILES=$(find "$LOG_DIR" -name "*.log*" -type f -mtime +$KEEP_DAYS 2>/dev/null | wc -l)

print_info "日志目录: $LOG_DIR"
print_info "当前大小: ${CURRENT_SIZE} MB"
print_info "总文件数: $TOTAL_FILES"
print_info "超过 $KEEP_DAYS 天的文件: $OLD_FILES"
echo ""

if [ "$DRY_RUN" = true ]; then
    print_warn "=== 干运行模式，不会实际删除文件 ==="
    echo ""
fi

# 删除所有日志
if [ "$DELETE_ALL" = true ]; then
    print_warn "将删除所有日志文件!"
    echo ""
    
    FILES_TO_DELETE=$(find "$LOG_DIR" -name "*.log*" -type f 2>/dev/null)
    
    if [ -z "$FILES_TO_DELETE" ]; then
        print_info "没有日志文件需要删除"
        exit 0
    fi
    
    echo "将删除以下文件:"
    echo "$FILES_TO_DELETE" | while read -r file; do
        size=$(du -h "$file" 2>/dev/null | cut -f1)
        echo "  - $file ($size)"
    done
    echo ""
    
    if [ "$DRY_RUN" = true ]; then
        print_info "干运行完成，未删除任何文件"
    else
        read -p "确认删除? (y/N): " confirm
        if [[ "$confirm" =~ ^[Yy]$ ]]; then
            find "$LOG_DIR" -name "*.log*" -type f -delete
            print_success "已删除所有日志文件"
        else
            print_info "已取消"
        fi
    fi
    exit 0
fi

# 检查是否需要清理
if [ "$CURRENT_SIZE" -lt "$SIZE_THRESHOLD_MB" ] && [ "$OLD_FILES" -eq 0 ]; then
    print_success "日志目录大小 (${CURRENT_SIZE}MB) 未超过阈值 (${SIZE_THRESHOLD_MB}MB)，无需清理"
    exit 0
fi

# 清理过期日志
DELETED_COUNT=0
DELETED_SIZE=0

print_info "开始清理超过 $KEEP_DAYS 天的日志文件..."
echo ""

# 找出要删除的文件
FILES_TO_DELETE=$(find "$LOG_DIR" -name "*.log*" -type f -mtime +$KEEP_DAYS 2>/dev/null)

if [ -z "$FILES_TO_DELETE" ]; then
    print_info "没有超过 $KEEP_DAYS 天的日志文件"
else
    echo "将删除以下文件:"
    echo "$FILES_TO_DELETE" | while read -r file; do
        if [ -f "$file" ]; then
            size=$(du -h "$file" 2>/dev/null | cut -f1)
            mtime=$(stat -c %y "$file" 2>/dev/null | cut -d' ' -f1)
            echo "  - $file ($size, $mtime)"
        fi
    done
    echo ""
    
    if [ "$DRY_RUN" = true ]; then
        DELETED_COUNT=$(echo "$FILES_TO_DELETE" | wc -l)
        print_info "干运行: 将删除 $DELETED_COUNT 个文件"
    else
        echo "$FILES_TO_DELETE" | while read -r file; do
            if [ -f "$file" ]; then
                rm -f "$file"
                ((DELETED_COUNT++)) || true
            fi
        done
        
        DELETED_COUNT=$(echo "$FILES_TO_DELETE" | grep -c . || echo 0)
        print_success "已删除 $DELETED_COUNT 个文件"
    fi
fi

# 清理空的滚动日志（如 .log.1, .log.2 等）
echo ""
print_info "检查空文件..."
EMPTY_FILES=$(find "$LOG_DIR" -name "*.log*" -type f -empty 2>/dev/null)

if [ -n "$EMPTY_FILES" ]; then
    EMPTY_COUNT=$(echo "$EMPTY_FILES" | wc -l)
    print_info "发现 $EMPTY_COUNT 个空文件"
    
    if [ "$DRY_RUN" = false ]; then
        find "$LOG_DIR" -name "*.log*" -type f -empty -delete
        print_success "已删除空文件"
    fi
fi

# 清理崩溃dump文件（超过7天）
echo ""
print_info "检查崩溃dump文件..."
CRASH_FILES=$(find "$LOG_DIR" -name "*crash*.log" -o -name "*terminate*.log" -type f -mtime +$KEEP_DAYS 2>/dev/null)

if [ -n "$CRASH_FILES" ]; then
    CRASH_COUNT=$(echo "$CRASH_FILES" | grep -c . || echo 0)
    print_info "发现 $CRASH_COUNT 个过期崩溃文件"
    
    if [ "$DRY_RUN" = false ]; then
        echo "$CRASH_FILES" | xargs rm -f 2>/dev/null || true
        print_success "已删除崩溃dump文件"
    fi
fi

# 显示清理后状态
echo ""
echo "=========================================="
NEW_SIZE=$(get_dir_size_mb "$LOG_DIR")
NEW_FILES=$(count_files "$LOG_DIR" "*.log*")

print_info "清理完成!"
print_info "清理前: ${CURRENT_SIZE} MB, $TOTAL_FILES 个文件"
print_info "清理后: ${NEW_SIZE} MB, $NEW_FILES 个文件"
print_info "释放空间: $((CURRENT_SIZE - NEW_SIZE)) MB"
echo "=========================================="
