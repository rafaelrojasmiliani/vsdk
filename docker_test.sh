
docker run --volume $(pwd):/test --user $(id -u):$(id -g) gsplines bash test.sh

if [ $1 = "install" ]; then
    docker run --volume $(pwd):/test --user $(id -u):$(id -g) gsplines \
    pip3 install --user git+https://github.com/rafaelrojasmiliani/vsdk.git
fi
