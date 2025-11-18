#include "XPlaneUDP.hpp"

using namespace std;

int main () {
    auto xp = XPlaneUdp();
    // 获取数据
    const auto latitude = xp.addDataref("sim/flightmodel/position/latitude");
    const auto engine = xp.addDatarefArray("sim/flightmodel/engine/ENGN_N1_", 16);
    // 改变获取频率
    xp.changeDatarefFreq(engine, 20);
    // 获取
    float latValue;
    array<float, 16> n1Values{};
    xp.getDataref(latitude, latValue);
    xp.getDataref(engine, n1Values);
    // 设置
    vector<float> n1;
    xp.setDatarefArray("sim/flightmodel/engine/ENGN_N1_", n1);
    return 0;
}
