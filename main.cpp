#include "XPlaneUDP.hpp"

using namespace std;

int main () {
    auto xp = XPlaneUdp();
    // 获取数据
    const auto latitude = xp.addDataref("sim/flightmodel/position/latitude");
    const auto engine = xp.addDatarefArray("sim/flightmodel/engine/ENGN_N1_", 16);
    // 改变获取频率
    xp.changeDatarefFreq(engine, 20);
    // dataref
    float latValue;
    array<float, 16> n1Values{};
    xp.getDataref(latitude, latValue);
    xp.getDataref(engine, n1Values);
    // 设置
    const vector<float> n1{27, 28};
    xp.setDatarefArray("sim/flightmodel/engine/ENGN_N1_", n1);
    // 回调
    xp.setCallback([](const bool state) { cerr << "state change: " << state << endl; });
    // 基本信息
    xp.addPlaneInfo(20);
    XPlaneUdp::PlaneInfo info{};
    bool rev{};
    const std::string set{"sim/cockpit/radios/com1_freq_hz"};
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // 获取基本信息
        xp.getPlaneInfo(info);
        xp.getDataref(latitude, latValue);
        cout << latValue << " " << info.lon << endl;
        // 写入数据
        rev = !rev;
        xp.setDataref(set, rev ? 12640 : 12665);
    }
    return 0;
}
