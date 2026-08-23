# V1.9.0.1 Stream Destination Hotfix

The Windows GCS sends a `pc_ip` field in HELLO, but that field is computed from
the PC's default route. On a machine with Wi-Fi + Ethernet + VPN + WSL/virtual
adapters, it can identify an interface that is not the interface used to reach
the Raspberry Pi.

V1.9.0.1 therefore treats the actual UDP HELLO source address (`peer[0]`) as
the authoritative video/metadata destination.

On connection the Pi now prints:

```text
[CONTROL] GCS CONNECTED peer=192.168.x.x claimed=...
[STREAM] destination 192.168.x.x | front=14560/14561 | bottom=14563/14564
[VIDEO-TX] MPEG-TS/H264 -> 192.168.x.x:14560 ...
[VIDEO-TX] MPEG-TS/H264 -> 192.168.x.x:14563 ...
```

If frames are still absent after this, compare the printed peer destination with
the Windows PC address and then inspect Windows firewall/UDP reception.
