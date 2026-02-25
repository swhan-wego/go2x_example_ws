#!/bin/bash

# CycloneDDS를 위한 네트워크 인터페이스 자동 설정 스크립트
# CYCLONEDDS_URI 환경 변수에 현재 시스템의 네트워크 인터페이스를 자동으로 설정합니다.
# 사용법: source setup_netif.sh
# 
# 이 스크립트는:
# 1. 현재 시스템의 네트워크 인터페이스 목록을 확인
# 2. 사용자가 선택할 수 있게 리스트를 보여줌
# 3. 사용자가 번호로 선택하여 적용
# 4. 일정 시간(10초) 동안 반응이 없으면 IP가 192로 시작하거나 eth로 시작하는 첫 인터페이스를 자동 선택

set -e

# 네트워크 인터페이스 목록 가져오기 (loopback 제외)
mapfile -t INTERFACES < <(ip link show | grep "^[0-9]" | awk '{print $2}' | sed 's/:$//' | grep -v "^lo$")

if [ ${#INTERFACES[@]} -eq 0 ]; then
    echo "❌ 사용 가능한 네트워크 인터페이스가 없습니다."
    return 1 2>/dev/null || exit 1
fi

echo "========================================"
echo "🔧 CycloneDDS 네트워크 인터페이스 설정"
echo "========================================"
echo ""
echo "📍 사용 가능한 네트워크 인터페이스:"
echo ""

# 인터페이스 목록 출력
for i in "${!INTERFACES[@]}"; do
    iface="${INTERFACES[$i]}"
    # 인터페이스의 IP 주소 조회
    ip_addr=$(ip addr show "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' | head -1 | cut -d'/' -f1 || echo "N/A")
    echo "  [$((i+1))] $iface (IP: $ip_addr)"
done

echo ""
echo "👉 사용할 인터페이스 번호를 입력하세요 (10초 타임아웃):"
echo ""

# 사용자 입력 대기 (10초 타임아웃)
selected_index=-1
if read -t 10 -p "번호 입력: " selected_input; then
    # 입력값이 유효한 번호인지 확인
    if [[ "$selected_input" =~ ^[0-9]+$ ]] && [ "$selected_input" -ge 1 ] && [ "$selected_input" -le ${#INTERFACES[@]} ]; then
        selected_index=$((selected_input - 1))
    else
        echo "❌ 잘못된 입력입니다. 첫 번째 유효한 인터페이스를 사용합니다."
    fi
else
    echo ""
    echo "⏱️  타임아웃! 자동 선택 중..."
fi

# 타임아웃 또는 유효하지 않은 입력인 경우, 자동 선택
if [ "$selected_index" -eq -1 ]; then
    for i in "${!INTERFACES[@]}"; do
        iface="${INTERFACES[$i]}"
        ip_addr=$(ip addr show "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' | head -1 | cut -d'/' -f1 || echo "")
        
        # 192로 시작하는 IP 주소를 찾음
        if [[ "$ip_addr" == 192* ]]; then
            selected_index=$i
            echo "✅ 자동 선택: $iface (IP: $ip_addr)"
            break
        fi
    done
    
    # 여전히 선택되지 않았다면, eth로 시작하는 첫 번째 인터페이스 선택
    if [ "$selected_index" -eq -1 ]; then
        for i in "${!INTERFACES[@]}"; do
            iface="${INTERFACES[$i]}"
            if [[ "$iface" == eth* ]]; then
                selected_index=$i
                echo "✅ 자동 선택: $iface"
                break
            fi
        done
    fi
    
    # 여전히 선택되지 않았다면 첫 번째 인터페이스 사용
    if [ "$selected_index" -eq -1 ]; then
        selected_index=0
        echo "✅ 자동 선택: ${INTERFACES[$selected_index]} (첫 번째 인터페이스)"
    fi
fi

# 선택된 인터페이스 적용
selected_interface="${INTERFACES[$selected_index]}"

# CYCLONEDDS_URI 생성
export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General><Interfaces><NetworkInterface name=\"${selected_interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

echo ""
echo "========================================"
echo "✅ 설정 완료!"
echo "========================================"
echo ""
echo "📡 선택된 인터페이스: $selected_interface"
echo "🔗 CYCLONEDDS_URI 설정됨"
echo ""
echo "현재 환경변수:"
echo "  CYCLONEDDS_URI='$CYCLONEDDS_URI'"
echo ""
echo "💡 이 설정은 현재 쉘 세션에만 적용됩니다."
echo "   영구적으로 적용하려면 ~/.bashrc 또는 ~/.zshrc에 추가하세요."
echo ""

set +e