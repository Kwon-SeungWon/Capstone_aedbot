import os
import datetime
import playsound
from pytz import timezone
import requests
import multiprocessing
import time
import subprocess


TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
KST = timezone("Asia/Seoul")

URL = "http://130.162.152.119"
FACETIME_URL = "https://crov.site:3000/"

CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))
SOUND_PATH = os.path.join(CURRENT_PATH, "sound")

SIREN_PATH = os.path.join(SOUND_PATH, "siren.mp3")
BPM_PATH = os.path.join(SOUND_PATH, "120bpm.mp3")

ARRIVE_PATH = os.path.join(SOUND_PATH, "when_arrive_tts.mp3")
START_CPR_PATH = os.path.join(SOUND_PATH, "start_cpr_tts.mp3")
FINISH_CPR_PATH = os.path.join(SOUND_PATH, "when_cpr_finish_tts.mp3")

# when_arrive_tts.mp3 => 로봇의 우측에서 CPR 팔찌를, 로봇의 후면에서 AED와 구급키트를 꺼내주세요
# start_cpr_tts.mp3 => 소리에 맞춰서 심폐소생술을 실시해주세요
# when_cpr_finish_tts.mp3 => 모든 상황이 종료되었습니다. 물품들을 원위치해주시고 복귀 버튼을 눌러주세요


def main():
    proc_arrive = multiprocessing.Process(
        target=playsound.playsound, args=(ARRIVE_PATH, True)
    )
    proc_arrive.start()
    proc_arrive.join()

    proc_start_cpr = multiprocessing.Process(
        target=playsound.playsound, args=(START_CPR_PATH, True)
    )
    proc_start_cpr.start()
    proc_start_cpr.join()

    proc_bpm = multiprocessing.Process(
        target=playsound.playsound, args=(BPM_PATH, True)
    )
    proc_bpm.start()

    time.sleep(3)

    proc_cpr_finish = multiprocessing.Process(
        target=playsound.playsound, args=(FINISH_CPR_PATH, True)
    )

    # while True:
    #     if get_time_diff(URL + "/get_quit"):
    #
    #         proc_cpr_finish.start()
    #         proc_cpr_finish.join()
    #         break

    #     if len(multiprocessing.active_children()) == 0:
    #         proc_bpm.start()

    #     time.sleep(0.01)

    # return None


if __name__ == "__main__":
    main()
