"""Jalankan drl.uji_robot_nyata tanpa jendela OpenCV (simulasi di latar belakang).

    cd "Forklift Robot Program" && python drl/uji_tanpa_jendela.py --skenario uji_nyata/Gudang.json --simulasi --tanpa-konfirmasi --ulang 2 --perencana astar,dqfd-murni

cv2.waitKey diganti tidur selama waktu yang sama, jadi loop 20 Hz tetap berjalan
pada laju yang sama; tidak ada tombol yang ditekan (mode --simulasi --tanpa-konfirmasi
selesai sendiri setelah percobaan terakhir). Dijalankan dari folder 'Forklift Robot Program'.
"""
import os
import runpy
import sys
import time

import cv2

cv2.namedWindow = lambda *a, **k: None
cv2.imshow = lambda *a, **k: None
cv2.destroyAllWindows = lambda *a, **k: None


def _tunggu(ms=0):
    time.sleep(max(0, ms) / 1000.0)
    return -1


cv2.waitKey = _tunggu

if __name__ == "__main__":
    sys.path.insert(0, os.getcwd())
    sys.argv = ["drl.uji_robot_nyata"] + sys.argv[1:]
    runpy.run_module("drl.uji_robot_nyata", run_name="__main__", alter_sys=True)
