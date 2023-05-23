import requests

test_data = {"cam_id": 0, "x": 0, "y": 0, "z": 0, "w": 0}

URL = "http://130.162.152.119"


if __name__ == "__main__":
    requests.post(URL + "/upload_dest", json=test_data)
    # requests.get(URL + "/arrive")
    # requests.get(URL + "/quit")
