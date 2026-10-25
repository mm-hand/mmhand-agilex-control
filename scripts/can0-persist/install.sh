#!/usr/bin/env bash
# Persistent, self-recovering SocketCAN bring-up for the Nero arms.
#
# What this fixes:
# 1. gs_usb (USB-CAN) adapters re-appear DOWN after every replug or reboot.
#    The units installed here bring each channel up at 1 Mbit/s whenever the
#    device appears, so nothing has to be done by hand.
# 2. Unplugging the CAN cable while the arm program is running wedges the
#    adapter in ERROR-PASSIVE / BUS-OFF, and it never recovers on its own:
#    this adapter REJECTS restart-ms (kernel: "Device doesn't support
#    restart from Bus Off"), so bus-off recovery means bouncing the link.
#    Replugging the bus cable does not re-run this unit either, because the
#    USB device never disappears - hence the next point.
# 3. hardware.try_reset_can_interface() bounces these units automatically
#    when a connect attempt finds the interface wedged; the passwordless
#    sudo rule installed here lets that succeed without a password prompt.
#    restart-ms is still set best-effort for adapters whose firmware does
#    support it.
#
# Run as the user (agents run with NoNewPrivs and cannot sudo):
#   sudo bash scripts/can0-persist/install.sh [channels...]
# Default channels: can0 can1 (can1 is harmless if that adapter is absent).
set -euo pipefail

channels="${*:-can0 can1}"
user="${SUDO_USER:-$(id -un)}"
[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

write_unit() {
    local channel="$1"
    cat > "/etc/systemd/system/${channel}.service" <<EOF
[Unit]
Description=Bring up SocketCAN interface ${channel} at 1 Mbit/s (AgileX Nero / gs_usb)
Documentation=man:systemd.device(5) man:ip-link(8)
BindsTo=sys-subsystem-net-devices-${channel}.device
After=sys-subsystem-net-devices-${channel}.device

[Service]
Type=oneshot
RemainAfterExit=yes
# "-" ignores failure when the link is already down
ExecStart=-/usr/sbin/ip link set ${channel} down
ExecStart=/usr/sbin/ip link set ${channel} type can bitrate 1000000
# Best effort only: gs_usb firmware rejects this ("Device doesn't support
# restart from Bus Off"); bus-off recovery then relies on bouncing this unit.
ExecStart=-/usr/sbin/ip link set ${channel} type can restart-ms 100
ExecStart=/usr/sbin/ip link set ${channel} up

[Install]
WantedBy=sys-subsystem-net-devices-${channel}.device
EOF
}

sudoers_rules="# analytic_control: bounce the CAN bring-up units without a password"
for channel in $channels; do
    write_unit "$channel"
    systemctl enable "${channel}.service" >/dev/null
    sudoers_rules+=$'\n'"${user} ALL=(root) NOPASSWD: /usr/bin/systemctl restart ${channel}.service"
done

echo "$sudoers_rules" > /etc/sudoers.d/analytic-can-reset
chmod 0440 /etc/sudoers.d/analytic-can-reset
visudo -cf /etc/sudoers.d/analytic-can-reset

systemctl daemon-reload
for channel in $channels; do
    # a previous failed start must not block the restart below
    systemctl reset-failed "${channel}.service" 2>/dev/null || true
    if [ -e "/sys/class/net/${channel}" ]; then
        systemctl restart "${channel}.service"
        ip -details link show "$channel" | grep -E 'state|bitrate|restart' || true
    fi
done

cat <<EOF

Verify (bus-cable replug, the case that used to wedge the adapter):
  1) run the arm program, pull the CAN cable, wait, plug it back in
  2) reconnect - hardware.py bounces ${channels} automatically; no terminal
     restart needed anymore (manual fallback: sudo systemctl restart can0.service)

Installed units: ${channels}
Remove with:
  sudo systemctl disable --now $(printf '%s.service ' ${channels})
  sudo rm -f $(printf '/etc/systemd/system/%s.service ' ${channels}) /etc/sudoers.d/analytic-can-reset
  sudo systemctl daemon-reload
EOF
