# sdrpp-inmarsatc-demodulator
Inmarsat-C TDM demodulator and decoder plugin for SDR++

Designed to provide output to stdcdec_parser and qstdcdec

Related projects:

    inmarsatc: library with all functions to receive Inmarsat-C signals
    https://github.com/cropinghigh/inmarsatc

    stdcdec: set of programs to receive inmarsat-c signals
    https://github.com/cropinghigh/stdcdec

    qstdcdec: qt version of stdcdec_parser
    https://github.com/cropinghigh/qstdcdec

Binary installing:

Visit the Actions page, find latest commit build artifacts, download inmarsatc_demodulator.so and put it to /usr/lib/sdrpp/plugins/, skipping to the step 3. Don't forget to install libinmarsatc!

Building:

  1.  Install SDR++ core headers to /usr/include/sdrpp_core/, if not installed. Refer to https://cropinghigh.github.io/sdrpp-moduledb/headerguide.html about how to do that

      OR if you don't want to use my header system, add -DSDRPP_MODULE_CMAKE="/path/to/sdrpp_build_dir/sdrpp_module.cmake" to cmake launch arguments

      Install inmarsatc library(https://github.com/cropinghigh/inmarsatc) if not installed(libinmarsatc-git package for arch-like systems):

          git clone https://github.com/cropinghigh/inmarsatc
          cd inmarsatc
          mkdir build
          cd build
          cmake ..
          make
          sudo make install

  2.  Build(from this project repo folder):

          mkdir build
          cd build
          cmake ..
          make
          sudo make install

  3.  Enable new module by adding it via Module manager

Usage:

  1.  Tune VFO to an Inmarsat-C signal

  2.  Wait for the sync, you'll see two points on the real axis and green sync indicator

  3.  After some time, you'll see decoded frames numbers and BER data.

  4.  Run stdcdec_parser(or other program to parse inmarsat-c frames like qstdcdec) and press "Start" button in the module.
