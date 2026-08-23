def ack(tx_id,ok,message='',data=None): return {'type':'ACK','tx_id':tx_id,'ok':bool(ok),'message':message,'data':data}
