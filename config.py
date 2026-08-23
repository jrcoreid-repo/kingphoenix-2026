from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
CONFIG_DIR=ROOT/'config'
def load_json(name):
    with (CONFIG_DIR/name).open('r',encoding='utf-8') as f: return json.load(f)
def save_json_atomic(name,data):
    p=CONFIG_DIR/name; t=p.with_suffix(p.suffix+'.tmp')
    with t.open('w',encoding='utf-8') as f: json.dump(data,f,indent=2)
    t.replace(p)
