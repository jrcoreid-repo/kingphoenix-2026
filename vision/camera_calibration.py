import json
from config import CONFIG_DIR
def load_camera_calibration(camera):
    p=CONFIG_DIR/'calibration'/f'{camera}_camera.json'
    with p.open('r',encoding='utf-8') as f:d=json.load(f)
    if not d.get('valid',False):raise RuntimeError(f'{camera} camera calibration is not valid')
    return d
