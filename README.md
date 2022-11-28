PDAP

Instructions:


apt install build-essential libboost-all-dev

On Ubuntu 20.04:
apt install gcc-10 g++-10 cpp-10
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10
pip install meson

meson setup --buildtype debug  build_debug 
meson setup --buildtype debugoptimized  build_debugoptimized 
meson setup --buildtype release  build_release
meson configure build_release/ -Db_ndebug=true
./compile.sh

Note that on 20.04 gcc and meson are old versions, on many other OS's the package manager should provide new enough versions by default!