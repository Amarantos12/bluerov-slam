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

echo_and_run cd "/home/hzr/catkin_ws/src/sonar-SLAM/bruce_slam"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hzr/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hzr/catkin_ws/install/lib/python3/dist-packages:/home/hzr/catkin_ws/build/bruce_slam/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hzr/catkin_ws/build/bruce_slam" \
    "/usr/bin/python3" \
    "/home/hzr/catkin_ws/src/sonar-SLAM/bruce_slam/setup.py" \
     \
    build --build-base "/home/hzr/catkin_ws/build/bruce_slam" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hzr/catkin_ws/install" --install-scripts="/home/hzr/catkin_ws/install/bin"
