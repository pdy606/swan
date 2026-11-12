"""ML-only subprocess: accepts six RGB images and a relative goal, never ROS."""
import argparse
import socket

from .mbra_adapter import MbraAdapter
from .protocol import receive_message, send_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket-fd', type=int, required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--provider', choices=['cpu', 'cuda'], default='cpu')
    parser.add_argument('--threads', type=int, default=2)
    args = parser.parse_args()
    connection = socket.socket(fileno=args.socket_fd)
    try:
        adapter = MbraAdapter(args.model, args.provider, args.threads)
        send_message(connection, {'type': 'ready', 'model': adapter.info})
        while True:
            request = receive_message(connection)
            if request['type'] == 'close':
                break
            if request['type'] != 'infer':
                raise ValueError('Unsupported worker request')
            try:
                result = adapter.infer(request['frames'], request['goal_xy'])
                send_message(connection, {'type': 'result', 'id': request['id'],
                                          'generation': request['generation'], **result})
            except (ValueError, RuntimeError) as error:
                send_message(connection, {'type': 'error', 'id': request['id'],
                                          'generation': request['generation'], 'error': str(error)})
    except (EOFError, BrokenPipeError, ConnectionResetError):
        pass
    except Exception as error:
        try:
            send_message(connection, {'type': 'fatal', 'error': str(error)})
        except OSError:
            pass
        raise
    finally:
        connection.close()


if __name__ == '__main__':
    main()
