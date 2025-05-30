from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quori_face'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    

        # This line makes sure the launch files are installer.  This will copy
        # all the files in the launch directory to the install location.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zmeyer',
    maintainer_email='zane.meyer@gmail.com',
    description='This creates a face for the Quori 1 Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_face = quori_face.draw_face:draw_face',
            'spin_eyes = quori_face.spin_eyes:main',
            'focal_point = quori_face.focal_point:main',
            'gaze_controller = quori_face.gaze_controller:main',
            'watch_color = quori_face.watch_color:main',
        ],
    },
)
