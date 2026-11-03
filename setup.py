import os
from glob import glob

from setuptools import find_packages, setup

PKG = "ksti_mav"


def tree(src):
    """Salin folder (models/) apa adanya ke share/ksti_mav/<src>."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join("share", PKG, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=PKG,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        ("share/" + PKG + "/launch", glob("launch/*.py")),
        ("share/" + PKG + "/config", glob("config/*")),
        ("share/" + PKG + "/worlds", glob("worlds/*.sdf")),
        *tree("models"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Deo",
    maintainer_email="mobilesantuy@gmail.com",
    description="Mobil otonom 1:10 untuk KSTI 2026 Mini Autonomous Vehicle",
    license="MIT",
    entry_points={
        "console_scripts": [
            "autopilot = ksti_mav.nodes.autopilot_node:main",
            "camera = ksti_mav.nodes.camera_node:main",
            "serial_bridge = ksti_mav.nodes.serial_bridge_node:main",
            "referee = ksti_mav.nodes.referee_node:main",
            "sim_drive = ksti_mav.nodes.sim_drive_node:main",
        ],
    },
)
