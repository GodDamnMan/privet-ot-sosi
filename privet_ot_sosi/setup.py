from setuptools import find_packages, setup

package_name = 'privet_ot_sosi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mobile',
    maintainer_email='mobile@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'encoder_driver={package_name}.encoder_driver:main',
            f"fake_encoder = {package_name}.fake_encoder:main"
        ],
    },
)
