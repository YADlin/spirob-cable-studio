# SSH from the Ubuntu workstation to the Windows/WSL laptop

Use Windows OpenSSH as the laptop's entry point. This allows native Windows commands and entry into the existing WSL distribution without publishing a separate WSL SSH port or maintaining NAT port-proxy rules. The robot coordinator remains local to the laptop; the workstation supplies terminal access.

These instructions assume the two computers can reach each other on the same trusted LAN. Some university/guest Wi-Fi networks isolate clients; if port 22 cannot be reached despite correct service/firewall setup, have the network administrator provide a permitted connection. Do not open a router's internet-facing SSH port for this setup.

## 1. Windows laptop: enable the server

Open **PowerShell as Administrator**:

```powershell
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
Start-Service sshd
Set-Service -Name sshd -StartupType Automatic
Get-Service sshd
```

Expected service state: Running. If installation cannot download the optional feature, install OpenSSH Server through Windows Settings > Optional features and retry the service commands; use your organization's approved Windows Update source.

Restrict the standard SSH firewall rule to local subnets:

```powershell
$spirobRule = Get-NetFirewallRule -Name OpenSSH-Server-In-TCP -ErrorAction SilentlyContinue
if ($spirobRule) {
    Set-NetFirewallRule -Name OpenSSH-Server-In-TCP -Enabled True -RemoteAddress LocalSubnet
} else {
    New-NetFirewallRule -Name OpenSSH-Server-In-TCP -DisplayName 'OpenSSH local subnet' -Enabled True -Direction Inbound -Protocol TCP -LocalPort 22 -RemoteAddress LocalSubnet -Action Allow
}
```

This permits the workstation on the local network and the WSL virtual network. It does not disable the firewall. Existing organization policy may impose additional restrictions.

Record the Windows account, laptop LAN IPv4 and SSH host fingerprint:

```powershell
whoami
Get-NetIPConfiguration | Format-List InterfaceAlias,IPv4Address,IPv4DefaultGateway
ssh-keygen -l -f "$env:ProgramData\ssh\ssh_host_ed25519_key.pub"
wsl --list --verbose
```

Use the IPv4 address of the connected Wi-Fi/Ethernet adapter, not the vEthernet WSL adapter. Use the actual Windows account, which may differ from the Linux `user` inside WSL. Note the exact WSL distribution name; it might be `Ubuntu` or `Ubuntu-24.04`.

## 2. Ubuntu workstation: first login

```bash
sudo apt update
sudo apt install -y openssh-client
read -r -p 'Laptop LAN IPv4 address: ' spirob_laptop_ip
read -r -p 'Windows SSH account name: ' spirob_windows_user
ssh -t -l "$spirob_windows_user" "$spirob_laptop_ip"
```

Compare the presented host fingerprint with the fingerprint printed on the laptop before accepting it. The password is the Windows account password, not its Windows Hello PIN. If the account has no usable password, configure the public key locally in Step 4 first. Microsoft/domain accounts can require a qualified login name; use `-l` to preserve it as one argument.

The initial remote shell is usually Windows Command Prompt. Enter:

```text
wsl --list --verbose
wsl -d Ubuntu-24.04
```

Replace only the distribution name with the one shown by the list. WSL distributions are registered per Windows account; logging in as a different Windows administrator may show an empty list.

You are now in the laptop's Ubuntu environment. Check:

```bash
whoami
hostname
cd ~/Research_Work/Two_Cable_HW/spirob-cable-studio
uv run python -m spirob_experiment --help
```

A headless experiment does not require X11 forwarding. Do not run the Qt GUI over SSH for acquisition. Launch the Windows camera agent locally first, then use SSH to enter WSL and launch the tunnel/coordinator. Keep separate terminal sessions for the long-running camera tunnel and coordinator. Ctrl+C in the coordinator requests stop/cleanup. Do not detach a hardware trial as an unattended background process.

To leave WSL, type `exit`; type it again to close the Windows SSH session.

## 3. One command to enter WSL later

From the Ubuntu workstation, using the variables set above:

```bash
ssh -t -l "$spirob_windows_user" "$spirob_laptop_ip" wsl.exe -d Ubuntu-24.04 -- bash -l
```

Use the verified distribution name. If Windows pseudo-terminal integration rejects this form on your installation, use the two-stage interactive login from Step 2. Both lead to the same existing WSL account and repository.

## 4. Add key authentication

On the workstation, create a separate key so an existing Git key is not overwritten:

```bash
test -f "$HOME/.ssh/id_ed25519_spirob" || ssh-keygen -t ed25519 -f "$HOME/.ssh/id_ed25519_spirob" -C spirob-workstation
cat "$HOME/.ssh/id_ed25519_spirob.pub"
```

Use a passphrase and your SSH agent if desired. Copy the entire single public-key line to the laptop. Keep the private key on the workstation.

On Windows, the destination depends on the account you will log into. Follow **one** branch below.

### Windows account is a standard user

Open ordinary PowerShell as that account:

```powershell
$spirobPublicKey = Read-Host 'Paste the complete ssh-ed25519 public key'
New-Item -ItemType Directory -Force "$env:USERPROFILE\.ssh" | Out-Null
Add-Content -Encoding ascii -Path "$env:USERPROFILE\.ssh\authorized_keys" -Value $spirobPublicKey
```

The file must be owned by that user and not writable by other ordinary users. If authentication rejects it, inspect its ACL and Windows OpenSSH logs instead of making it broadly writable.

### Windows account is an administrator

With the default Windows sshd configuration, administrator accounts use the shared administrators key file. Open PowerShell as Administrator:

```powershell
$spirobPublicKey = Read-Host 'Paste the complete ssh-ed25519 public key'
$spirobKeyFile = "$env:ProgramData\ssh\administrators_authorized_keys"
Add-Content -Encoding ascii -Path $spirobKeyFile -Value $spirobPublicKey
icacls.exe $spirobKeyFile /inheritance:r /grant '*S-1-5-32-544:F' /grant '*S-1-5-18:F'
```

Check for any pre-existing explicit grants to other principals; the administrators key file should be accessible only to Administrators and SYSTEM. A key in this shared file authorizes administrator-account logins, so use a dedicated standard account if you need narrower access. If sshd has been customized, follow its actual `AuthorizedKeysFile` and `Match` rules.

On the workstation, test:

```bash
ssh -i "$HOME/.ssh/id_ed25519_spirob" -t -l "$spirob_windows_user" "$spirob_laptop_ip"
```

WSL's local camera tunnel is a separate SSH client. It can initially use the Windows account password; for repeated tests, generate a separate WSL key and authorize its public half using the same Windows instructions. Do not copy the workstation private key to the laptop.

## 5. Connections and files

The data root in the acquisition guide is the Windows profile's `SpiRob_Data` directory. Both WSL and native Windows see the same files. You can copy a closed session to the workstation using `scp` or SFTP over this SSH connection; paths follow the Windows SSH server's filesystem view. Confirm the path interactively before scripting copies.

Do not turn on Google Drive synchronization of the live recording directory yet. The future upload module will consume closed sessions and their checksums.

## Troubleshooting

- **Timeout/refused:** check Windows `Get-Service sshd`, correct LAN IP, port-22 firewall rule, network client isolation and VPN routing.
- **Permission denied:** verify the Windows username, password versus PIN, and which authorized-keys file applies to that account.
- **No WSL distributions:** use the Windows account that owns the installed distribution.
- **Motors absent after restart:** reattach CP2102 to WSL with the already documented usbipd procedure; SSH does not perform USB passthrough.
- **Permission denied on ttyUSB0:** use the existing dialout membership; a new SSH/WSL login should receive current groups. Check `id -nG`.
- **Camera-check timeout:** verify the Windows camera agent, WSL tunnel and token file; the agent is intentionally bound only to loopback.
- **Laptop suspends:** use Windows power settings to keep it awake during an attended experiment. Suspend breaks USB and timing continuity; restart the session afterward.

SSH is terminal access, not an independent emergency-stop channel. Retain the physical motor-power disconnect.

## Official references

- https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse
- https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_keymanagement
- https://learn.microsoft.com/en-us/windows/wsl/networking
