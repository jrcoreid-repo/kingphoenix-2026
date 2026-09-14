# Kingphoenix Models

Runtime models live under `models/` so Raspberry Pi clones can use repository-relative paths instead of machine-specific `/home/...` paths.

## Hailo-8 detector

Planned active model:

- Repository path: `models/hailo8/yolo26_gate_v2.hef`
- Source upload name: `yolo26_baru.hef`
- Size: 7,606,771 bytes
- SHA-256: `f19dacab1740c0eede457498e44ba7796a70655d6c28625af0bfdd30868aea44`
- Input size: 640

`vision/hailo_yolo26.py` supports both absolute legacy paths and repository-relative paths. Relative paths are resolved from the Kingphoenix repository root.

## Safe activation rule

Do not change `config/system.json -> ai.detector.hef` to the new repository path until the HEF binary is present in this repository and its SHA-256 has been verified on the Raspberry Pi.

After the binary is present, set:

```json
"hef": "models/hailo8/yolo26_gate_v2.hef"
```

Then on Raspberry Pi verify with:

```bash
sha256sum models/hailo8/yolo26_gate_v2.hef
```

Expected SHA-256:

```text
f19dacab1740c0eede457498e44ba7796a70655d6c28625af0bfdd30868aea44
```
