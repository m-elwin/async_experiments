from setuptools import setup

package_name = 'async_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','launch/experiment.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Elwin',
    maintainer_email='elwin@northwestern.edu',
    description='Experiments in python asynchronous services',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delay_server = async_experiments.delay_server:delay_entry',
            'deadlock = async_experiments.async_client:deadlock_entry',
            'await = async_experiments.async_client:await_entry',
            'future = async_experiments.async_client:future_entry',
            'yield = async_experiments.async_client:yield_entry'
        ],
    },
)
