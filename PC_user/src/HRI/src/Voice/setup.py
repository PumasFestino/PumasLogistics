from setuptools import find_packages, setup

package_name = 'Voice'

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
    maintainer='vero',
    maintainer_email='veronica.morales.tepale@gmail.com',
    description='Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Aquí agregamos tus nodos
            'voice_talker = Voice.voice_talker:main',
            'voice_listener = Voice.voice_listener:main',
        ],
    },
)

