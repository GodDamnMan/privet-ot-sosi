from setuptools import setup

py_pkg_name  = "privet_ot_sosi"   
ros_pkg_name = py_pkg_name
setup(
    name=ros_pkg_name,
    version="0.0.0",
    packages=[py_pkg_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + ros_pkg_name]),
        ("share/" + ros_pkg_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"my_node = {py_pkg_name}.fake_encoder:main",
        ],
    },
)
