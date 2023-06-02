# CV-PY

## Features

This crate includes a `dev` feature that is used to switch between using images from Mirte's
camera to `/assets/input_1.jpg`. This feature can be specified by passing the `-F dev` flag to a
build command as you can see [here](../.gitlab-ci.yml#41).

By default, this feature is *disabled* which means that the input will come from Mirte's camera.
This will be useful in the next section where compiling is discussed.

## Compiling

You need Python (>= 3.7) and [Maturin](https://www.maturin.rs/installation.html) installed.

```sh
sudo apt-get install python3.7 python3-pip -y
pip3 install maturin patchelf
```

For development, create and activate a Python environment. 
[The docs](https://www.maturin.rs/tutorial.html#install-and-configure-maturin-in-a-virtual-environment)
suggest running the following:

```sh
# Recommended you change to this directory as the venv is gitignored
cd cv-py/

# You might not need this and you might also need to specify a different 3.x number, try running
# the next command first and see if you get an error. If you do, you should get the apt install
# you need to run included in your error.
sudo apt install python3.8-venv
python3 -m venv .venv
source .venv/bin/activate # You will need to run this on every new shell to enable the environment.
pip install -U pip maturin maturin patchelf 
```

You should then be able to build the Python bindings. The following command builds and installs the
Python wheel:

```sh
# In the root folder
maturin develop -m cv-py/Cargo.toml -r

# Or from the `cv-py` folder
maturin develop -r
```

This should install this crate's Python bindings in the Python interpreter of your virtual
environment. To verify that it works you can run `python3 cv-py/test.py` which runs the
`detect_line_type` method. 

> Note that as of now, the test uses a relative path on the Rust side to import an input image from
> the `assets` folder. For this reason, you need to run this command from the root so as to not get
> a file not found error. This test file is meant to be a demonstration and when we switch to using
> in memory images from the camera it will obviously not be an issue.

For release, run:

```sh
maturin build -m cv-py/Cargo.toml --skip-auditwheel -r
# The wheel should be in ./target/wheels/

pip install ./target/wheels/cv_py-0.1.0-cp38-cp38-linux_x86_64.whl
# For reinstalling, add a `--force` flag at the end of the pip install command
```

## Type Hints

A [Python Interface](./cv_py.pyi) file is provided that adds type hints which can then be used
by code editors to add intellisence. These can unfortunately not be generated automatically for the
time being and thus need to be kept up to date with the underlying code manually. Note that if
they are outdated, the library will still be usable, just the type hints will be
misleading/incomplete.

The type hints should be visible to your IDE just by installing the library itself
(`maturin develop` refreshes them as well for example).
