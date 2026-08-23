class AbortHandler:
    def __init__(self,state): self.state=state
    def reset_to_standby(self,reason):
        with self.state.lock:
            self.state.mission.status='STANDBY'; self.state.mission.step_index=0; self.state.mission.active_ai_camera='none'; self.state.mission.abort_reason=reason
