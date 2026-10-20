"""
drl - perencana jalur DQfD (Deep Q-learning from Demonstrations) pengganti
Hybrid A*. Rancangannya di catatan Obsidian "Deep Reinforcement Learning".

FULL DQfD (2026-10-20): demonstratornya Hybrid A* + Reeds-Shepp, tapi saat
dipakai seluruh jalur dipilih jaringan - tanpa Reeds-Shepp maupun A*.

    lingkungan.py    MDP perencanaan: primitif penuh + pendek, masking, biaya, observasi
    adaptor_demo.py  jalur Hybrid A* -> episode demonstrasi, + uji penerimaan

Dijalankan dari folder "Forklift Robot Program", sebagai modul:
    python -m drl.adaptor_demo --uji
"""
