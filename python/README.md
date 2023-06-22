# The Python Stuff ™️

This folder contains Python helper code for interacting with our ROS topics.

## Running

Note that `computer_vision.py` needs the `mirte-rs` project to be running since it is fetching data
from our ROS topics. You can run `cargo r --example image_feed ./assets/input_1.jpg` for that.
Refer to [this README](../ros/README.md) for details on how to get that to work.

```sh
cd python
python3 -m venv .venv

source .venv/bin/activate
pip install -r requirements.txt
python3 computer_vision.py
```

## Installing the Wheel

```sh
python3 -m pip install --upgrade build
python3 -m build
pip3 install dist/mirte-1.0-py3-none-any.whl
```
