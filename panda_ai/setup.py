from setuptools import find_packages, setup

package_name = 'panda_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kumar Utkarsh',
    maintainer_email='kutkarsh706@gmail.com',
    description='Gemini-powered Tic Tac Toe AI node for Franka Panda robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_maker = panda_ai.decision_maker:main',
        ],
    },
)
