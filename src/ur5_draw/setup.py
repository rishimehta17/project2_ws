from setuptools import setup

package_name = "ur5_draw"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="UR5 drawing pipeline without MoveIt2",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "simple_mover = ur5_draw.simple_mover:main",
            "live_plotter_multiview = ur5_draw.live_plotter_multiview:main",
            "direct_drawer = ur5_draw.direct_drawer:main",
            "web_direct_controller = ur5_draw.web_direct_controller:main",
        ],
    },
)
