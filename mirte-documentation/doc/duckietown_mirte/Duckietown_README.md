# Mirte Duckietown getting docs working

```sh
cd mirte-documentation
python3 -m venv docs-env
source docs-env/bin/activate
pip install -r requirements.txt
sudo apt-get install enchant -y
pip install -r ../python/requirements.txt
cd _modules/mirte-python
pip install .
cd ../catkin_ws/src/mirte-ros-packages
rm -rfv !("mirte_msgs")
cd ../../
catkin_make # or catkin build
source devel/setup.bash
cd ../../
```

Now do everything in the installation guide  under `INSTALL MIRTE DUCKIETOWN MESSAGES`

```sh
make html
```
