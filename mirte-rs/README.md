# Let the Mirte robot drive around in Duckietown

Source code of the Mirte Duckietown Software Project.

## Compiling

We are targeting Ubuntu 20.04, most of us are running that through WSL.

The steps for compiling each crate are listed in detail on each crate's README.

In addition to that, a [Dockerfile](./Dockerfile) is provided which includes all dependencies
needed, we are using that in our CI workflow.

### Compiling as ROS Node

This folder is a ROS package. In order to run it, it must be within your `catkin` workspace. You can either copy the folder there, or link it by running:

```
ln -s <path-to-repo>/mirte-rs <catkin-workspace>/src/mirte-rs
```

Note:\
- For most people, `<catkin-workspace>` is `~/catkin_ws`
- `<path-to-repo>` needs to be an absolute path, otherwise linking fails

After linking or copying, go to your `catkin` workspace directory and run `catkin_make`. Then, you can run the node two ways:
- `rosrun mirte-rs mirte-rs` (first name is package name, second is executable name. In our case, these are the same. Also, you need to have `roscore` running to use `rosrun`.)
- `roslaunch mirte-rs duckietown_navigator.launch` (This launches `roscore` automatically)\

In both cases, you can run `rosnode info duckietown_navigator` to verify the node is running.

## Cross Compiling

Compiling on the Mirte bot itself is terribly inefficient which is why you might want to compile
on your workstation, *for* Mirte. A [docker image](./Dockerfile.cross) is provided. Running the
following will build the image, run it and open an interactive shell for you:

```sh
docker build . -t 'xcompile' -f Dockerfile.cross
docker run --platform linux/arm64/v8 --name 'xcompile' -it xcompile:latest bash
```

You can then run any Rust build command to produce an output binary such as:

```sh
# Build the project's entry point in release
cargo build --release
```

> Note that the time it takes for this build to complete can vary severely and depends on your
> host machine's operating system, architecture and amount of RAM. It took us anywhere between
> 15 minutes to 90 minutes. This is trivial to add to your CI pipeline and
> [publish an artifact](https://github.com/actions/upload-artifact#upload-an-individual-file),
> we haven't done that as of now since the project will be moved from Gitlab to Github soon.

This resulting binary should be in `./target/release/mirte-rs`. To copy it over to your host
machine, you can run 

```sh
docker cp xcompile:~/target/release/mirte-rs .
```

You can now transfer the file over to Mirte via SSH

```sh
scp ./mirte-rs mirte@<Mirte's IP>:/home/mirte/
```

## Cargo Docs

You can generate the documentation by running

```sh
$ cargo doc --no-deps
```

Normally, you can pass the `--open` flag which would open the docs in your browser. If you are
however running this from WSL, that won't work and you'll need to switch to Powershell/CMD.

If the project is in the Windows filesystem then that should be easy. If it is instead in the WSL
file system, `cd` into `//wsl$/Ubuntu/` which is the root of WSL, then navigate into the project
directory and open `./target/doc/cv/index.html`.

> You can also run this as one command, for example:
> 
> ```sh
> \\wsl$\Ubuntu\home\let-the-mirte-robot-drive-around-in-duckietown\target\doc\cv\index.html
> ```
> opens the docs for the [`cv`](./cv/) crate

