#!/usr/bin/env bash
set -euo pipefail

# ====== CONFIG ======
ROBOT_IP="${ROBOT_IP:-192.168.123.104}"
HOST_IP_CIDR="${HOST_IP_CIDR:-192.168.123.10/24}"

# Можно передать интерфейс первым аргументом: ./setup_ur_net.sh enx1234...
IFACE="${1:-}"

# ====== DEPS ======
need_cmd() { command -v "$1" >/dev/null 2>&1; }

pkg_mgr() {
  if need_cmd apt-get; then echo "apt"; return; fi
  if need_cmd dnf; then echo "dnf"; return; fi
  if need_cmd pacman; then echo "pacman"; return; fi
  if need_cmd apk; then echo "apk"; return; fi
  echo ""
}

install_pkgs() {
  local mgr="$1"; shift
  case "$mgr" in
    apt)
      export DEBIAN_FRONTEND=noninteractive
      apt-get update -y
      apt-get install -y --no-install-recommends "$@"
      ;;
    dnf)
      dnf install -y "$@"
      ;;
    pacman)
      pacman -Sy --noconfirm "$@"
      ;;
    apk)
      apk add --no-cache "$@"
      ;;
    *)
      return 1
      ;;
  esac
}

ensure_deps() {
  local missing=()
  for c in "$@"; do
    need_cmd "$c" || missing+=("$c")
  done

  if ((${#missing[@]} == 0)); then
    return 0
  fi

  echo "Missing commands: ${missing[*]}" >&2

  local mgr
  mgr="$(pkg_mgr)"
  if [[ -z "$mgr" ]]; then
    echo "No supported package manager found. Install missing tools manually." >&2
    exit 1
  fi

  echo "Attempting to install missing tools using: $mgr" >&2

  # Маппинг: команда -> пакеты (по менеджеру)
  # ip обычно из iproute2; ping из iputils (Debian/Ubuntu) / iputils (Arch) / iputils-ping (Fedora/RHEL)
  local pkgs=()

  for c in "${missing[@]}"; do
    case "$c" in
      ip)
        case "$mgr" in
          apt) pkgs+=("iproute2") ;;
          dnf) pkgs+=("iproute") ;;
          pacman) pkgs+=("iproute2") ;;
          apk) pkgs+=("iproute2") ;;
        esac
        ;;
      ping)
        case "$mgr" in
          apt) pkgs+=("iputils-ping") ;;
          dnf) pkgs+=("iputils") ;;          # ping обычно тут
          pacman) pkgs+=("iputils") ;;
          apk) pkgs+=("iputils") ;;
        esac
        ;;
      awk)
        case "$mgr" in
          apt) pkgs+=("gawk") ;;
          dnf) pkgs+=("gawk") ;;
          pacman) pkgs+=("gawk") ;;
          apk) pkgs+=("gawk") ;;
        esac
        ;;
      *)
        # неизвестная команда — не знаем, какой пакет ставить
        echo "Don't know which package provides '$c'. Install it manually." >&2
        exit 1
        ;;
    esac
  done

  # Уберём дубликаты пакетов
  if ((${#pkgs[@]} > 0)); then
    mapfile -t pkgs < <(printf "%s\n" "${pkgs[@]}" | awk '!seen[$0]++')
  fi

  if ! install_pkgs "$mgr" "${pkgs[@]}"; then
    echo "Failed to install packages: ${pkgs[*]}" >&2
    exit 1
  fi

  # финальная проверка
  local still_missing=()
  for c in "${missing[@]}"; do
    need_cmd "$c" || still_missing+=("$c")
  done
  if ((${#still_missing[@]} > 0)); then
    echo "Still missing after install: ${still_missing[*]}" >&2
    exit 1
  fi
}

# Требуем root, чтобы ставить пакеты/конфигурить сеть
if [[ "$EUID" -ne 0 ]]; then
  echo "Please run as root: sudo $0 [iface]" >&2
  exit 1
fi

# Минимальный набор утилит для скрипта
ensure_deps ip ping awk

echo "Robot IP:      $ROBOT_IP"
echo "Host address:  $HOST_IP_CIDR"

# Если интерфейс не указан — попробуем найти сами
if [[ -z "$IFACE" ]]; then
  echo "Searching for Ethernet interfaces (en*)..."
  CANDIDATES=$(ip -br link | awk '$1 !~ /^lo$/ && $1 ~ /^en/ {print $1}')

  if [[ -z "$CANDIDATES" ]]; then
    echo "No Ethernet interfaces (en*) found. Pass interface explicitly, e.g. sudo $0 enx123456" >&2
    exit 1
  fi
else
  CANDIDATES="$IFACE"
fi

FOUND_IF=""

for IF in $CANDIDATES; do
  echo "Trying interface: $IF"

  # Сбрасываем старые адреса и задаём новый
  ip addr flush dev "$IF" || true
  ip addr add "$HOST_IP_CIDR" dev "$IF"
  ip link set "$IF" up

  sleep 1

  echo "Pinging robot $ROBOT_IP from $IF..."
  if ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "SUCCESS: interface $IF can reach $ROBOT_IP"
    FOUND_IF="$IF"
    break
  else
    echo "No response from $ROBOT_IP via $IF"
  fi
done

if [[ -z "$FOUND_IF" ]]; then
  echo "ERROR: No interface could reach robot at $ROBOT_IP." >&2
  exit 1
fi

echo
echo "Final interface state:"
ip -br addr show "$FOUND_IF"

echo
echo "Done. You can now use this interface to talk to the Livox Mid-70."