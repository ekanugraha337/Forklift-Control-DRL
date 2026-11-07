"""
drl - perencana jalur DQfD (Deep Q-learning from Demonstrations) pengganti
Hybrid A*. Rancangannya di catatan Obsidian "Struktur DQfD Full".

ATURAN ver-1-6: satu model per robot (model jauh) yang meniru Hybrid A* + RS
(demonstrator w = 2,0, h = max(h_RS, h_D), RS < 60 cm). Saat dipakai model jauh
memilih primitif sampai di dalam 60 cm dan tidak ada halangan, lalu tembakan
Reeds-Shepp menutup jalur ke tujuan. Pelatihan: gradien PyTorch, forward numpy;
32 sinar dengan numba.

    lingkungan.py      MDP perencanaan: primitif, masking, biaya, serah + tembakan RS, observasi
    adaptor_demo.py    jalur Hybrid A* -> episode demonstrasi, + uji penerimaan
    skenario.py        pembangkit dataset demo
    latih.py           pelatihan DQfD (dqfd.py, jaringan.py, jaringan_numpy.py, rollout.py)
    perencana_dqfd.py  pencarian berpandu Q + DQfDPlanner untuk rencana.Perencana

Dijalankan dari folder "Forklift Robot Program", sebagai modul:
    python -m drl.adaptor_demo --uji
"""
