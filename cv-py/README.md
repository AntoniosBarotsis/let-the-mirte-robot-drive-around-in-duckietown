# CV-PY

## Compiling

You need Python (>= 3.8) and [Maturin](https://www.maturin.rs/installation.html) installed.

```sh
sudo apt-get install python3.7 python3-pip -y
pip3 install maturin patchelf
```

For development, create and activate a Python environment, then run:

```sh
maturin develop -m cv-py/Cargo.toml
```

This should install this crate's Python bindings in the Python interpreter of your virtual
environment. To verify that it works you can run `python3 cv-py/test.py` which runs the
`detect_line_type` method.

For release, run:

```sh
maturin build -m cv-py/Cargo.toml --skip-auditwheel -r
# The wheel should be in ./target/wheels/

pip install ./target/wheels/cv_py-0.1.0-cp38-cp38-linux_x86_64.whl
# For reinstalling, add a `--force` flag at the end of the pip install command
```
