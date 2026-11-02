#!/usr/bin/env python3
"""Temporary, guest-only relay from a UTM VM to Mac's loopback Ollama API."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen


class Relay(BaseHTTPRequestHandler):
    allowed_client = '192.168.64.5'

    def do_POST(self):
        if self.client_address[0] != self.allowed_client or self.path != '/api/chat':
            self.send_error(403)
            return
        length = int(self.headers.get('Content-Length', '0'))
        if length < 1 or length > 16384:
            self.send_error(413)
            return
        body = self.rfile.read(length)
        try:
            request = Request('http://127.0.0.1:11434/api/chat', data=body,
                              headers={'Content-Type': 'application/json'}, method='POST')
            with urlopen(request, timeout=20) as upstream:
                data = upstream.read(65536)
        except Exception as error:
            self.send_error(502, type(error).__name__)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='192.168.64.1')
    parser.add_argument('--port', type=int, default=11435)
    parser.add_argument('--allow-client', default='192.168.64.5')
    args = parser.parse_args()
    Relay.allowed_client = args.allow_client
    server = ThreadingHTTPServer((args.bind, args.port), Relay)
    print(f'Ollama relay on {args.bind}:{args.port}, allowing {args.allow_client}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
