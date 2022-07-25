# sdrpp-inmarsatc-demodulator
Inmarsat-C TDM demodulator plugin for SDR++

Designed to provide output to stdcdec_decoder and qstdcdec(planned)

Building:
  1.  Install SDR++ core headers to /usr/include/sdrpp_core/(if not installed) (sdrpp-headers-git package for arch-like systems)

          git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
          cd "SDRPlusPlus/core/src"
          sudo mkdir -p "/usr/include/sdrpp_core"
          sudo find . -regex ".*\.\(h\|hpp\)" -exec cp --parents \{\} "/usr/include/sdrpp_core" \;

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
  3.  Run stdcdec_decoder and press "Start" button. If you've selected "--verbose" mode, you'll see decoded frames
