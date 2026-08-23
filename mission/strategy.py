VALID={'LEFT','RIGHT'}
def normalize_strategy(v):
    s=str(v).upper()
    if s not in VALID: raise ValueError(v)
    return s
