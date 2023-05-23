import os
import datetime
import playsound
from pytz import timezone
import requests
import multiprocessing
import time


TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")

URL = "http://130.162.152.119"

CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))
SOUND_PATH = os.path.join(CURRENT_PATH, "sound")

SIREN_PATH = os.path.join(SOUND_PATH, "siren.mp3")
BPM_PATH = os.path.join(SOUND_PATH, "120bpm.mp3")


def get_time_diff(URL):
    r = requests.get(url=URL)
    data = r.json()
    server_time = data["time"]

    current_time = datetime.datetime.now(KST).strftime(TIME_FORMAT)
    time_diff = datetime.datetime.strptime(
        current_time, TIME_FORMAT
    ) - datetime.datetime.strptime(server_time, TIME_FORMAT)

    if abs(time_diff.total_seconds()) < 5:
        return True

    return False


def main():
    while True:
        if get_time_diff(URL + "/get_dest"):
            break
        time.sleep(0.01)

    # # os.system("pactl -- set-sink-volume 0 100%")
    proc_bpm = multiprocessing.Process(
        target=playsound.playsound, args=(SIREN_PATH, True)
    )
    proc_bpm.start()

    while True:
        if get_time_diff(URL + "/get_arrive"):
            proc_bpm.terminate()
            break

        if len(multiprocessing.active_children()) == 0:
            proc_bpm.start()

        time.sleep(0.01)

    # os.system("pactl -- set-sink-volume 0 100%")
    proc_siren = multiprocessing.Process(
        target=playsound.playsound, args=(BPM_PATH, True)
    )
    proc_siren.start()

    while True:
        if get_time_diff(URL + "/get_quit"):
            proc_siren.terminate()
            break

        if len(multiprocessing.active_children()) == 0:
            proc_siren.start()

        time.sleep(0.01)

    return None


if __name__ == "__main__":
    main()
