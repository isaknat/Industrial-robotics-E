"""Run a project CAD stage through the local Autodesk Fusion MCP."""
import json
from pathlib import Path
import sys
from urllib.request import Request, urlopen

HERE = Path(__file__).resolve().parent
headers = {'Accept':'application/json, text/event-stream','Content-Type':'application/json'}
def send(method, params, request_id=None):
    obj = dict(jsonrpc='2.0',method=method,params=params)
    if request_id:
        obj['id'] = request_id
    with urlopen(Request('http://127.0.0.1:27183/mcp',json.dumps(obj).encode(),headers),timeout=1800) as response:
        if method == 'initialize':
            headers['MCP-Session-Id'] = response.headers['MCP-Session-Id']
        data = response.read()
        return json.loads(data) if data else None

send('initialize',dict(protocolVersion='2025-03-26',capabilities={},clientInfo=dict(name='CAD-v3',version='1')),1)
send('notifications/initialized',{})
catalog = send('tools/list',{},2)['result']['tools']
if len(sys.argv)==1:
    print([(t['name'],list(t['inputSchema'].get('properties',{}))) for t in catalog])
    sys.exit()
stage = sys.argv[1]
assert stage.isidentifier()
script = "def run(_context: str):\n    import runpy\n    runpy.run_path("+repr((HERE/'edition3.py').as_posix())+")["+repr(stage)+"]()"
name = next(t['name'] for t in catalog if 'featureType' in t['inputSchema'].get('properties',{}) and 'script' in t['description'])
result=send('tools/call',dict(name=name,arguments=dict(featureType='script',object=dict(script=script))),3)
if 'error' in result or result.get('result',{}).get('isError'):
    print(result)
    sys.exit(1)
payload=json.loads(result['result']['content'][0]['text'])
print(payload.get('message') or payload.get('error') or payload)
sys.exit(0 if payload.get('success') else 1)
