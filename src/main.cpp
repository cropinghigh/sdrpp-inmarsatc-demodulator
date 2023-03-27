#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <utils/flog.h>
#include <utils/net.h>

#include <dsp/demod/psk.h>
#include <dsp/buffer/reshaper.h>
#include <dsp/buffer/packer.h>
#include <dsp/routing/splitter.h>
#include <dsp/sink/handler_sink.h>
#include <dsp/convert/complex_to_real.h>
#include <dsp/digital/binary_slicer.h>

#include <gui/widgets/constellation_diagram.h>

#include <inmarsatc_decoder.h>

#define ENABLE_SYNC_DETECT

#include "symbol_extractor.h"
#include "gui_widgets.h"

#define CONCAT(a, b) ((std::string(a) + b).c_str())

#define VFO_SAMPLERATE 2400
#define CLOCK_RECOVERY_BW 0.01f
#define CLOCK_RECOVERY_DAMPN_F 0.71f
#define CLOCK_RECOVERY_REL_LIM 0.001f
#define RRC_TAP_COUNT 33
#define RRC_ALPHA 0.35f
#define AGC_RATE 0.02f
#define COSTAS_LOOP_BANDWIDTH 0.01f

SDRPP_MOD_INFO {
    /* Name:            */ "inmarsatc_demodulator",
    /* Description:     */ "Inmarsat-c demodulator for SDR++",
    /* Author:          */ "cropinghigh",
    /* Version:         */ 0, 3, 1,
    /* Max instances    */ -1
};

ConfigManager config;

class InmarsatcDemodulatorModule : public ModuleManager::Instance {
public:
    InmarsatcDemodulatorModule(std::string name) : frameDec(9) {
        this->name = name;

        // Load config
        config.acquire();
        if (!config.conf.contains(name)) {
            config.conf[name]["hostname"] = hostname;
            config.conf[name]["port"] = port;
            config.conf[name]["listening"] = false;
        }
        std::string host = config.conf[name]["hostname"];
        strcpy(hostname, host.c_str());
        port = config.conf[name]["port"];
        config.release(true);

        float recov_bandwidth = CLOCK_RECOVERY_BW;
        float recov_dampningFactor = CLOCK_RECOVERY_DAMPN_F;
        float recov_denominator = (1.0f + 2.0*recov_dampningFactor*recov_bandwidth + recov_bandwidth*recov_bandwidth);
        float recov_mu = (4.0f * recov_dampningFactor * recov_bandwidth) / recov_denominator;
        float recov_omega = (4.0f * recov_bandwidth * recov_bandwidth) / recov_denominator;
        mainDemodulator.init(nullptr, 1200, VFO_SAMPLERATE, RRC_TAP_COUNT, RRC_ALPHA, AGC_RATE, COSTAS_LOOP_BANDWIDTH, recov_omega, recov_mu, CLOCK_RECOVERY_REL_LIM);
        constDiagSplitter.init(&mainDemodulator.out);
        constDiagSplitter.bindStream(&constDiagStream);
        constDiagSplitter.bindStream(&demodStream);
        constDiagReshaper.init(&constDiagStream, 1024, 0);
        constDiagSink.init(&constDiagReshaper.out, _constDiagSinkHandler, this);

        symbolExtractor.init(&demodStream);
        symbolPacker.init(&symbolExtractor.out, 5000); //pack symbols to groups of 5000, as inmarsat-c decoder requires
        demodSink.init(&symbolPacker.out, _demodSinkHandler, this);

        enable();

        if(config.conf[name]["listening"]) {
            startServer();
        }

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~InmarsatcDemodulatorModule() {
        if(isEnabled()) {
            disable();
        }
        gui::menu.removeEntry(name);
    }

    void postInit() {}

    void enable() {
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, 2500, VFO_SAMPLERATE, 2500, 2500, true);
        mainDemodulator.setInput(vfo->output);
        mainDemodulator.start();
        constDiagSplitter.start();
        constDiagReshaper.start();
        constDiagSink.start();
        symbolExtractor.start();
        symbolPacker.start();
        demodSink.start();
        enabled = true;
    }

    void disable() {
        mainDemodulator.stop();
        constDiagSplitter.stop();
        constDiagReshaper.stop();
        constDiagSink.stop();
        symbolExtractor.stop();
        symbolPacker.stop();
        demodSink.stop();
        sigpath::vfoManager.deleteVFO(vfo);
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

private:

    static void menuHandler(void* ctx) {
        InmarsatcDemodulatorModule* _this = (InmarsatcDemodulatorModule*)ctx;

        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!_this->enabled) {
            style::beginDisabled();
        }
        ImGui::Text("Signal constellation: ");
        ImGui::SetNextItemWidth(menuWidth);
        _this->constDiag.draw();
#ifdef ENABLE_SYNC_DETECT
        float avg = 1.0f - _this->symbolExtractor.stderr;
        ImGui::Text("Signal quality: ");
        ImGui::SameLine();
        ImGui::SigQualityMeter(avg, 0.5f, 1.0f);
        ImGui::Text("Sync ");
        ImGui::SameLine();
        ImGui::BoxIndicator(menuWidth, _this->symbolExtractor.sync ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 5, 5, 255));
#endif
        ImGui::Text("Frames decoding result: ");
        if(_this->lastFrame.length != -1) {
            ImGui::TextColored(ImVec4(0.1, 0.8, 0.1, 1.0), "   Last frame number: ");
            ImGui::SameLine();
            ImGui::Text(std::to_string(_this->lastFrame.frameNumber).c_str());
            (_this->lastFrame.BER < 2) ? ImGui::TextColored(ImVec4(0.1, 0.8, 0.1, 1.0), "   BER: ") : ImGui::TextColored(ImVec4(0.8, 0.1, 0.1, 1.0), "   BER: ");
            ImGui::SameLine();
            ImGui::Text(std::to_string(_this->lastFrame.BER).c_str());
            if(_this->lastFrame.isReversedPolarity) ImGui::Text("   Reversed polarity");
            if(_this->lastFrame.isMidStreamReversePolarity) ImGui::TextColored(ImVec4(0.8, 0.8, 0.1, 1.0), "   Midstream rev polarity");
            if(_this->lastFrame.isUncertain) ImGui::TextColored(ImVec4(0.8, 0.8, 0.1, 1.0), "   Uncertain");
        } else {
            ImGui::TextColored(ImVec4(0.8, 0.1, 0.1, 1.0), "   No decoded frames");
        }

        bool listening = _this->conn && _this->conn->isOpen();

        if (listening && _this->enabled) { style::beginDisabled(); }
        if (ImGui::InputText(CONCAT("##_inmarsatc_demodulator_host_", _this->name), _this->hostname, 1023) && _this->enabled) {
            config.acquire();
            config.conf[_this->name]["hostname"] = _this->hostname;
            config.release(true);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputInt(CONCAT("##_inmarsatc_demodulator_port_", _this->name), &(_this->port), 0, 0) && _this->enabled) {
            config.acquire();
            config.conf[_this->name]["port"] = _this->port;
            config.release(true);
        }
        if (listening && _this->enabled) { style::endDisabled(); }
        if (listening && ImGui::Button(CONCAT("Stop##_network_sink_stop_", _this->name), ImVec2(menuWidth, 0)) && _this->enabled) {
            _this->stopServer();
            config.acquire();
            config.conf[_this->name]["listening"] = false;
            config.release(true);
        } else if (!listening && ImGui::Button(CONCAT("Start##_network_sink_stop_", _this->name), ImVec2(menuWidth, 0)) && _this->enabled) {
            _this->startServer();
            config.acquire();
            config.conf[_this->name]["listening"] = true;
            config.release(true);
        }
        ImGui::Text("Status:");
        ImGui::SameLine();
        if (_this->conn && _this->conn->isOpen()) {
            ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "Sending");
        } else if (listening) {
            ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "Listening");
        } else {
            ImGui::Text("Idle");
        }
        if (!_this->enabled) {
            style::endDisabled();
        }
    }

    void startServer() {
        stopServer();
        try {
            conn = net::openudp(hostname, port);
        } catch (std::runtime_error& e) {
            flog::error("Network error: %s\n", e.what());
        }
    }

    void stopServer() {
        if (conn) { conn->close(); }
    }

    static void _constDiagSinkHandler(dsp::complex_t* data, int count, void* ctx) {
        InmarsatcDemodulatorModule* _this = (InmarsatcDemodulatorModule*)ctx;

        dsp::complex_t* cdBuff = _this->constDiag.acquireBuffer();
        if(count == 1024) {
            memcpy(cdBuff, data, count * sizeof(dsp::complex_t));
        }
        _this->constDiag.releaseBuffer();
    }

    //serialization
    template< typename T >
    static std::array< char, sizeof(T) >  to_bytes( const T& object ) {
        std::array< char, sizeof(T) > bytes ;

        const char* begin = reinterpret_cast< const char* >( std::addressof(object) ) ;
        const char* end = begin + sizeof(T) ;
        std::copy( begin, end, std::begin(bytes) ) ;

        return bytes ;
    }

    template< typename T >
    static T& from_bytes( const std::array< char, sizeof(T) >& bytes, T& object ) {
        // http://en.cppreference.com/w/cpp/types/is_trivially_copyable
        static_assert( std::is_trivially_copyable<T>::value, "not a TriviallyCopyable type" ) ;

        char* begin_object = reinterpret_cast< char* >( std::addressof(object) ) ;
        std::copy( std::begin(bytes), std::end(bytes), begin_object ) ;

        return object;
    }


    static void _demodSinkHandler(uint8_t* data, int count, void* ctx) {
        InmarsatcDemodulatorModule* _this = (InmarsatcDemodulatorModule*)ctx;
        std::vector<inmarsatc::decoder::Decoder::decoder_result> results = _this->frameDec.decode(data);
        if(results.size() > 0) {
            _this->lastFrame = results[results.size()-1];
            for(int i = 0; i < results.size(); i++) {
                std::lock_guard lck(_this->connMtx);
                if(_this->conn && _this->conn->isOpen()) {
                    inmarsatc::decoder::Decoder::decoder_result res = results[i];
                    auto serialized = to_bytes(res);
                     _this->conn->send((uint8_t*)serialized.data(), serialized.size());
                }
            }
        }

    }

    std::string name;
    bool enabled = true;

    VFOManager::VFO* vfo;

    dsp::demod::InmarsatCDemod mainDemodulator;

    dsp::routing::Splitter<dsp::complex_t> constDiagSplitter;

    dsp::stream<dsp::complex_t> constDiagStream;
    dsp::buffer::Reshaper<dsp::complex_t> constDiagReshaper;
    dsp::sink::Handler<dsp::complex_t> constDiagSink;
    ImGui::ConstellationDiagram constDiag;

    dsp::stream<dsp::complex_t> demodStream;

    dsp::BPSKSymbolExtractor symbolExtractor;
    dsp::buffer::Packer<uint8_t> symbolPacker;
    dsp::sink::Handler<uint8_t> demodSink;
    inmarsatc::decoder::Decoder frameDec;
    inmarsatc::decoder::Decoder::decoder_result lastFrame = {{}, -1};

    std::shared_ptr<net::Socket> conn;
    std::mutex connMtx;
    char hostname[1024] = "localhost\0";
    int port = 15004;

};

MOD_EXPORT void _INIT_() {
    std::string root = (std::string)core::args["root"];
    json def = json({});
    config.setPath(root + "/inmarsatc_demodulator_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new InmarsatcDemodulatorModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (InmarsatcDemodulatorModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
