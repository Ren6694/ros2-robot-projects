#!/usr/bin/env bash
# =============================================================================
#  脚本: scripts/install_desktop_shortcut.sh   （在虚拟机里跑）
#
#  作用: 把 scripts/arm6-moveit-demo.desktop 装成**桌面上/应用列表里能点的东西**。
#        装完有三个入口，哪个顺手用哪个：
#          1) 应用网格（Activities → 打字"机械臂"）→ 点一下
#          2) ~/桌面 里的图标（GNOME 若开了"显示桌面图标"）
#          3) 左侧 Dock 固定（本脚本尝试，失败不影响前两个）
#
#  为什么要脚本而不是手 copy 一个文件（五条都是实测坑）：
#    a) .desktop 里的路径必须绝对，而 $HOME 只有装的时候才知道 ⇒ 用模板 +
#       sed 替换 @HOME@，不要把用户名写死进仓库。
#    b) GNOME 对"不受信任"的桌面图标**不给双击**（要么不显示，要么弹"是否信任"）。
#       信任位不在文件里，是 GIO 元数据 ⇒ 必须 `gio set … metadata::trusted true`。
#    c) 这份桌面环境是中文本地化，用户目录叫 ~/桌面 不叫 ~/Desktop。
#       所以用 `xdg-user-dir DESKTOP` 取真实路径，取不到再回退两个候选名。
#    d) ★Ubuntu 24.04 的桌面图标扩展（ding）对 .desktop **额外要求文件本身可执行**：
#       只设 trusted 而留 644，图标会带一个红色禁止角标、双击没反应，
#       标题也只显示文件名而不是 Name=（本轮截图里正是这个样子）。
#       ⇒ 桌面副本 chmod 755；~/.local/share/applications 那份按规范保持 644。
#    e) ★`update-desktop-database` 会往**它处理的那个目录**里写 mimeinfo.cache。
#       对桌面目录跑一次，用户桌面上就多出一个 mimeinfo.cache（本轮踩过）。
#       ⇒ 只对 applications 目录刷新。
#
#  幂等: 重复跑只是重装同一个文件，不会累积。
#  退出码: 0 = 至少装进了应用列表；1 = 连应用列表都没写成。
# =============================================================================
set -uo pipefail

WS="${WS:-$HOME/ros2_ws}"
TPL="$WS/scripts/arm6-moveit-demo.desktop"
LAUNCHER="$WS/scripts/one_shot_demo.sh"
NAME=arm6-moveit-demo.desktop

[ -f "$TPL" ] || { echo "SHORTCUT-INSTALL FAIL reason=template_missing path=$TPL"; exit 1; }
[ -f "$LAUNCHER" ] || { echo "SHORTCUT-INSTALL FAIL reason=launcher_missing path=$LAUNCHER"; exit 1; }
chmod +x "$LAUNCHER" 2>/dev/null || true

install_into() {           # $1=目录  $2=是否设 trusted  $3=是否可执行（桌面图标要）
    local dir="$1" trust="${2:-0}" exec_bit="${3:-0}" out="$1/$NAME"
    [ -d "$dir" ] || return 1
    sed "s|@HOME@|$HOME|g" "$TPL" > "$out" || return 1
    if [ "$exec_bit" = "1" ]; then
        chmod 755 "$out"          # ding 要求可执行，否则图标带红色禁止角标（坑 d）
    else
        chmod 644 "$out"
    fi
    if [ "$trust" = "1" ] && command -v gio >/dev/null 2>&1; then
        gio set "$out" metadata::trusted true 2>/dev/null || true
    fi
    echo "SHORTCUT-INSTALL ok path=$out perms=$([ "$exec_bit" = "1" ] && echo 755 || echo 644) trusted=$trust"
    return 0
}

RC=1
install_into "$HOME/.local/share/applications" 0 0 && RC=0
# 只对 applications 刷 mime 库（坑 e）；写不进也无所谓，图标不依赖它。
command -v update-desktop-database >/dev/null 2>&1 && \
    update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true

DESKDIR="$(command -v xdg-user-dir >/dev/null 2>&1 && xdg-user-dir DESKTOP 2>/dev/null)"
[ -n "${DESKDIR:-}" ] && [ -d "$DESKDIR" ] || DESKDIR=""
for d in "$DESKDIR" "$HOME/桌面" "$HOME/Desktop"; do
    [ -n "$d" ] || continue
    [ -d "$d" ] || continue
    install_into "$d" 1 1 && {
        # 上一版脚本在这里跑过 update-desktop-database，留下过一个无关文件；
        # 只清这个明确名字，不动用户桌面上别的东西。
        [ -f "$d/mimeinfo.cache" ] && rm -f "$d/mimeinfo.cache" && echo "SHORTCUT-INSTALL cleanup=removed_stale_mimeinfo_cache"
        break
    }
done

# Dock 固定：失败很正常（没有 gnome-shell / DBUS 不通），不影响主流程。
if command -v gsettings >/dev/null 2>&1; then
    CUR="$(gsettings get org.gnome.shell favorite-apps 2>/dev/null || true)"
    # 注意空列表的原文是 "@as []"（GVariant 打印格式），不是 "[]"：
    # 拿它走下面的"去掉右括号再拼"分支会拼出一个非法的 GVariant，
    # 所以单独一条分支直接写成只含我们的项。
    case "$CUR" in
        *"$NAME"*) echo "SHORTCUT-INSTALL dock=already_pinned" ;;
        '@as []') gsettings set org.gnome.shell favorite-apps "['$NAME']" 2>/dev/null \
                      && echo "SHORTCUT-INSTALL dock=pinned" \
                      || echo "SHORTCUT-INSTALL dock=skipped reason=gsettings_failed" ;;
        ''|*[!']') echo "SHORTCUT-INSTALL dock=skipped reason=no_favorite_list" ;;
        *)  NEW="$(printf '%s' "${CUR%]}" "'$NAME',]")"
            gsettings set org.gnome.shell favorite-apps "$NEW" 2>/dev/null \
                && echo "SHORTCUT-INSTALL dock=pinned" \
                || echo "SHORTCUT-INSTALL dock=skipped reason=gsettings_failed" ;;
    esac
fi

# 桌面图标扩展是否开着（决定 ~/桌面 那个副本可见不可见，只报状态不改设置）。
# ★别只 grep "desktop-icons"：Ubuntu 24.04 上这个扩展叫 ding@rastersoft.com，
#   只匹配前者会在一台**图标其实可见**的机器上报出 0，属于工具自己造出来的假结论。
if command -v gnome-extensions >/dev/null 2>&1; then
    DING="$(gnome-extensions list --enabled 2>/dev/null | grep -oE 'ding@rastersoft\.com|desktop-icons@[^ ]*' | head -1 || true)"
    if [ -n "$DING" ]; then
        echo "SHORTCUT-INSTALL desktop_icons=visible ext=$DING"
    else
        echo "SHORTCUT-INSTALL desktop_icons=hidden reason=no_desktop_icons_extension"
    fi
fi

if [ "$RC" -eq 0 ]; then
    echo "SHORTCUT-INSTALL done 入口=应用网格搜「机械臂」/ 桌面图标 / 一条命令 bash $LAUNCHER"
else
    echo "SHORTCUT-INSTALL FAIL reason=no_applications_dir"
fi
exit "$RC"
