import requests
import argparse

test_data = {"cam_id": 0, "x": 0, "y": 0, "z": 0, "w": 0}

URL = "http://130.162.152.119"


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--upload", action="store_true")
    args.add_argument("--arrive", action="store_true")
    args.add_argument("--quit", action="store_true")
    args = args.parse_args()

    if args.upload:
        requests.post(URL + "/upload_dest", json=test_data)

    if args.arrive:
        requests.get(URL + "/arrive")

    if args.quit:
        requests.get(URL + "/quit")
