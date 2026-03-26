#!/usr/bin/env python3
import argparse
import time
import carla

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--town", required=True)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()

    print(f"[target] connecting to CARLA {args.host}:{args.port}")
    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)

    print(f"[target] loading map {args.town}")
    world = client.load_world(args.town)

    print("[target] waiting for map to stabilize...")
    time.sleep(8.0)

    print(f"[target] ACTIVE MAP: {world.get_map().name}")

if __name__ == "__main__":
    main()
