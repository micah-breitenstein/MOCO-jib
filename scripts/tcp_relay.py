#!/usr/bin/env python3
import argparse
import socket
import threading


def pipe(src, dst):
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except Exception:
        pass
    finally:
        try:
            dst.shutdown(socket.SHUT_WR)
        except Exception:
            pass


def handle_client(client_sock, target_host, target_port):
    upstream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    upstream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    upstream.connect((target_host, target_port))

    t1 = threading.Thread(target=pipe, args=(client_sock, upstream), daemon=True)
    t2 = threading.Thread(target=pipe, args=(upstream, client_sock), daemon=True)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    client_sock.close()
    upstream.close()


def main():
    parser = argparse.ArgumentParser(description="Simple TCP relay")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=18080)
    parser.add_argument("--target-host", default="10.42.0.247")
    parser.add_argument("--target-port", type=int, default=80)
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen_host, args.listen_port))
    server.listen(100)
    print(f"relay listening on {args.listen_host}:{args.listen_port} -> {args.target_host}:{args.target_port}")

    while True:
        client, _ = server.accept()
        threading.Thread(
            target=handle_client,
            args=(client, args.target_host, args.target_port),
            daemon=True,
        ).start()


if __name__ == "__main__":
    main()
