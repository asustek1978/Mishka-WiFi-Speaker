#!/usr/bin/env python3
"""Diagnostic-only HTTPS raw-wire radio stream reader. Python 3 standard library.
Never paste signed media URLs or access tokens into shared logs.
Usage: python radio_wire_probe.py 'https://station.example/stream' --seconds 15
"""
import argparse, socket, ssl, time, urllib.parse
p=argparse.ArgumentParser();p.add_argument('url');p.add_argument('--seconds',type=float,default=15);args=p.parse_args()
u=urllib.parse.urlsplit(args.url)
if u.scheme not in ('https','http') or not u.hostname: p.error('HTTP(S) URL required')
port=u.port or (443 if u.scheme=='https' else 80)
path=urllib.parse.urlunsplit(('', '', u.path or '/',u.query,''))
s=socket.create_connection((u.hostname,port),timeout=6)
if u.scheme=='https': s=ssl.create_default_context().wrap_socket(s,server_hostname=u.hostname)
s.settimeout(2)
request=(f'GET {path} HTTP/1.1\r\nHost: {u.hostname}\r\nUser-Agent: Mishka-Raw-Wire-Probe/1.0\r\nAccept: */*\r\nAccept-Encoding: identity\r\nIcy-MetaData: 0\r\nConnection: close\r\n\r\n').encode()
s.sendall(request)
raw=b''
while b'\r\n\r\n' not in raw and len(raw)<65536: raw+=s.recv(4096)
if b'\r\n\r\n' not in raw: raise RuntimeError('HTTP header missing')
head,body=raw.split(b'\r\n\r\n',1)
for line in head.decode('latin1').split('\r\n'):
    if line.startswith('HTTP/') or line.lower().startswith(('transfer-encoding:', 'content-type:', 'content-length:', 'connection:', 'icy-br:')): print(line)
print('First raw body bytes (hex):',body[:48].hex(' '))
print('Note: raw body includes HTTP chunk-size lines when Transfer-Encoding is chunked')
start=time.monotonic(); total=len(body); last=start; gaps=0
while time.monotonic()-start<args.seconds:
    try:
        chunk=s.recv(8192)
        if not chunk: print('TCP EOF');break
        now=time.monotonic(); gap=now-last; last=now;total+=len(chunk)
        if gap>1: print('Read gap %.2f s, received %d bytes'%(gap,len(chunk)));gaps+=1
    except socket.timeout: print('Socket timeout, raw total',total)
print('Raw body bytes:',total,'elapsed:',round(time.monotonic()-start,2),'s','long gaps:',gaps)
s.close()
