## New things
```
sudo apt-get install pry bundler
cd mujoco_py
bundle install
./gen_binding.sh
```
Download: `ctypesgen` from [here](https://github.com/davidjamesca/ctypesgen)
```
python ctypesgen.py -l/work4/pulkitag-code/pkgs/mujoco/mjpro131/bin/libmujoco140nogl.so /work4/pulkitag-code/pkgs/mujoco/mjpro131/include/mujoco.h -o blah.py
```

# MuJoCo Python Bindings

MuJoCo is a physics engine which can do very detailed efficient
simulations with contacts. This library lets you use MuJoCo from
Python.

Python bindings for mujoco140

# Installing this library

You can install this library using:

```
pip install mujoco-py
```

MuJoCo isn't open-source, so you'll also need to set download the
MuJoCo binaries and obtain license key.

## Obtaining the binaries and license key

1. Obtain a 30-day free trial on the MuJoCo website:
   https://www.roboti.us/license.html. The license key will arrive in
   an email with your username and password.
2. Download the MuJoCo version 1.31 binaries for
   [Linux](https://www.roboti.us/active/mjpro131_linux.zip),
   [OSX](https://www.roboti.us/active/mjpro131_osx.zip), or
   [Windows](https://www.roboti.us/active/mjpro131_windows.zip).
3. Download your license key (the `mjkey.txt` file from your email)
   and unzip the downloaded mjpro bundle.
4. Place these in `~/.mujoco/mjkey.txt` and `~/.mujoco/mjpro131`. You
   can alternatively set the following environment variables:

```
export MUJOCO_PY_MJKEY_PATH=/path/to/mjkey.txt
export MUJOCO_PY_MJPRO_PATH=/path/to/mjpro131
```

## Testing

Run:

```
make test
```
