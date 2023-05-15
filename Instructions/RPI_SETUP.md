# Instructions to bring up and operate RPI

## SSH connection

To connect with the RPi in terminal via SSH, simply type:

```bash
ssh kgliwinski@secret_ip
```

After inputing the correct password the connection should be established

## VNC server connection

To connect via vnc, after establishing the ssh connection type:

```bash
vnserver -geometry 1920x1080 :1
```
-geometry sets the resolution of vnc window.

After getting info about successful vnc server establishment, on your PC run a VNC viewer app, i.e. Tiger VNC viewer, input the secret_ip:1 as the address, kgliwinski as username and correct password.

