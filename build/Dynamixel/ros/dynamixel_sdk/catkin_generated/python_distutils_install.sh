#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/mnt/c/users/david/Desktop/HexaPod/src/Dynamixel/ros/dynamixel_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/mnt/c/users/david/Desktop/HexaPod/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/mnt/c/users/david/Desktop/HexaPod/install/lib/python3/dist-packages:/mnt/c/users/david/Desktop/HexaPod/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/mnt/c/users/david/Desktop/HexaPod/build" \
    "/usr/bin/python3" \
    "/mnt/c/users/david/Desktop/HexaPod/src/Dynamixel/ros/dynamixel_sdk/setup.py" \
     \
    build --build-base "/mnt/c/users/david/Desktop/HexaPod/build/Dynamixel/ros/dynamixel_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/mnt/c/users/david/Desktop/HexaPod/install" --install-scripts="/mnt/c/users/david/Desktop/HexaPod/install/bin"
