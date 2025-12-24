#include "XPlaneUDP.hpp"
#include <cstdio>
#include <string>
#include <vector>
#include <stdexcept>
#include <charconv>

using namespace std;

class FastFileWriter {
    public:
        explicit FastFileWriter (const std::string &filename, const std::size_t bufferSize = 1 << 16)
            : bufferSize_(bufferSize), buffer_(new char[bufferSize]), pos_(0) {
            file_ = std::fopen(filename.c_str(), "wb");
            if (!file_) {
                throw std::runtime_error("Failed to open file: " + filename);
            }
        }
        void write (char c) {
            writeRaw(c);
            writeRaw(',');
        }
        void write (float v) {
            char buf[32];
            // 转换为固定格式，确保至少3位小数
            auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::fixed, 3);
            if (ec != std::errc()) {
                throw std::runtime_error("to_chars failed");
            }
            auto len = static_cast<std::size_t>(ptr - buf);
            writeRaw(buf, len);
            writeRaw(',');
        }
        void write (const std::vector<float> &v) {
            char buf[32];
            for (float x : v) {
                auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::fixed, 3);
                if (ec != std::errc()) {
                    throw std::runtime_error("to_chars failed");
                }
                auto len = static_cast<std::size_t>(ptr - buf);
                writeRaw(buf, len);
                writeRaw(',');
            }
        }
        void newline () {
            writeRaw('\n');
        }
        void flush () {
            if (pos_ > 0) {
                std::fwrite(buffer_, 1, pos_, file_);
                pos_ = 0;
            }
        }
        ~FastFileWriter () {
            if (file_) {
                flush();
                std::fclose(file_);
            }
            delete[] buffer_;
        }
    private:
        void writeRaw (char c) {
            buffer_[pos_++] = c;
            if (pos_ == bufferSize_) {
                flush();
            }
        }
        void writeRaw (const char *data, std::size_t len) {
            for (std::size_t i = 0; i < len; ++i) {
                writeRaw(data[i]);
            }
        }
        std::FILE *file_{nullptr};
        std::size_t bufferSize_;
        char *buffer_;
        std::size_t pos_;
};


int main () {
    auto xp = XPlaneUdp();
    // 获取Dataref
    constexpr int freq{30}; // 实际帧率会被限制在游戏帧率附近
    float timeValue{};
    const auto time = xp.addDataref("sim/time/zulu_time_sec", freq);
    vector<float> engineValue(16);
    const auto engine = xp.addDatarefArray("sim/flightmodel/engine/ENGN_N1_", 16, 120);
    vector<float> fuelValue(16);
    const auto fuel = xp.addDatarefArray("sim/cockpit2/engine/indicators/fuel_flow_kg_sec", 16, freq);
    vector<float> windValue(3);
    const auto wind = xp.addDatarefArray("sim/weather/wind_direction_degt", 3, freq);
    vector<float> isaValue(10);
    const auto isa = xp.addDatarefArray("sim/weather/temperatures_aloft_delta_ISA_C", 10, freq);
    vector<float> batteryValue(8);
    const auto battery = xp.addDatarefArray("sim/cockpit/electrical/battery_charge_watt_hr", 8, freq);
    vector<float> iceValue(16);
    const auto ice = xp.addDatarefArray("sim/cockpit/switches/anti_ice_inlet_heat_per_enigne", 16, freq);
    vector<float> ailValue(56);
    const auto ail = xp.addDatarefArray("sim/flightmodel/controls/ail1_def", 56, freq);
    // 获取基本信息
    xp.addPlaneInfo(freq);
    XPlaneUdp::PlaneInfo info{};
    // 设置Dataref
    bool rev{};
    const std::string set{"sim/cockpit/radios/com1_freq_hz"};
    // 回调
    xp.setCallback([](const bool state) { cerr << "state change: " << state << endl; });

    FastFileWriter fileWriter("xpPerformanceTest.csv");
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / freq)));
        // 获取
        xp.getPlaneInfo(info);
        xp.getDataref(time, timeValue);
        xp.getDataref(engine, engineValue);
        xp.getDataref(fuel, fuelValue);
        xp.getDataref(wind, windValue);
        xp.getDataref(isa, isaValue);
        xp.getDataref(battery, batteryValue);
        xp.getDataref(ice, iceValue);
        xp.getDataref(ail, ailValue);

        fileWriter.write(timeValue);
        fileWriter.write(engineValue);
        fileWriter.write(fuelValue);
        fileWriter.write(windValue);
        fileWriter.write(isaValue);
        fileWriter.write(batteryValue);
        fileWriter.write(iceValue);
        fileWriter.write(ailValue);
        fileWriter.newline();
        // 写入数据
        rev = !rev;
        xp.setDataref(set, rev ? 12540 : 12665);
    }
    return 0;
}
