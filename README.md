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

Building:

  1.  Install SDR++ core headers to /usr/include/sdrpp_core/(if not installed) (sdrpp-headers-git package for arch-like systems)

          git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
          cd "SDRPlusPlus/core/src"
          sudo mkdir -p "/usr/include/sdrpp_core"
          sudo find . -regex ".*\.\(h\|hpp\)" -exec cp --parents \{\} "/usr/include/sdrpp_core" \;

      Install inmarsatc library(https://github.com/cropinghigh/inmarsatc) if not installed(libinmarsatc-git package for arch-like systems).

          git clone https://github.com/cropinghigh/inmarsatc
          cd inmarsatc
          mkdir build
          cd build
          cmake ..
          make
          sudo make install

  2.  Build:

          mkdir build
          cd build
          cmake ..
          make
          sudo make install

  4.  Enable new module by adding

          "Inmarsat-C demodulator": {
            "enabled": true,
            "module": "inmarsatc_demodulator"
          }

      to config.json, or add it via Module manager

Usage:

  1.  Tune VFO to an Inmarsat-C signal

  2.  Wait for the sync, you'll see two points on the real axis and green sync indicator

  3.  After some time, you'll see decoded frames numbers and BER data.

  4.  Run stdcdec_parser(or other program to parse inmarsat-c frames like qstdcdec) and press "Start" button in the module.
