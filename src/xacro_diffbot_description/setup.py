from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xacro_diffbot_description'

# Define the source directory for launch files
# Assuming you place launch files in the 'launch' subdirectory
launch_dir = 'launch'
# Assuming you place URDF/Xacro files in the 'urdf' subdirectory
urdf_dir = 'urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 📌 1. REQUIRED: Install resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        # 📌 2. REQUIRED: Install package.xml
        ('share/' + package_name, ['package.xml']),
        
        # 📌 3. RECOMMENDED: Install all Xacro files (using the globbing you already had)
        (os.path.join('share', package_name, urdf_dir), glob(os.path.join(urdf_dir, '*.xacro'))),
        
        # 📌 4. RECOMMENDED: Install all launch files
        # This will install files like 'display.launch.py' into the install/share/<pkg>/launch directory
        (os.path.join('share', package_name, launch_dir), glob(os.path.join(launch_dir, '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='melvin',
    maintainer_email='melvintk09@gmail.com',
    description='ROS 2 package containing the diffbot Xacro robot description and launch files.',
    license='Apache-2.0', # Placeholder: Use a common open-source license
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # No Python nodes defined yet, so this remains empty
        ],
    },
)