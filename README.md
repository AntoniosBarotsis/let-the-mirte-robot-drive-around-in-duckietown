# Let the Mirte robot drive around in Duckietown

Source code of the Mirte Duckietown Software Project.

## Compiling

We are targeting Ubuntu 20.04, most of us are running that through WSL.

The steps for compiling each crate are listed in detail on each crate's README.

In addition to that, a [Dockerfile](./Dockerfile) is provided which includes all dependencies
needed, we are using that in our CI workflow.

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

