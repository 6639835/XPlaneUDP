#ifndef XPLANEUDP_HPP
#define XPLANEUDP_HPP

#include <boost/system.hpp>
#include <boost/asio.hpp>
#include <boost/dynamic_bitset.hpp>
#include <format>
#include <iostream>
#include <ranges>


template <typename T>
concept Container = std::ranges::random_access_range<T> &&
        std::ranges::sized_range<T> &&
        std::is_assignable_v<std::ranges::range_reference_t<T>, float> && requires(T contain) {
            contain.size();
        } ;

template <typename T>
concept CharContainer = requires(T contain) {
    requires std::same_as<typename T::value_type, char>;
    contain.data();
};

constexpr int HEADER_LENGTH{5}; // 指令头部长度 4字母+1空
constexpr static std::string DATAREF_GET_HEAD{'R', 'R', 'E', 'F', '\x00'};
constexpr static std::string DATAREF_SET_HEAD{'D', 'R', 'E', 'F', '\x00'};
constexpr static std::string BASIC_INFO_HEAD{'R', 'P', 'O', 'S', '\x00'};
constexpr static std::string BECON_HEAD{'B', 'E', 'C', 'N', '\x00'};

namespace sys = boost::system;
namespace asio = boost::asio;
namespace ip = asio::ip;

class XPlaneUdp;

template <typename T, typename... Rests>
    requires (std::same_as<std::string, T> || std::is_fundamental_v<T>)
size_t pack (std::vector<char> &container, size_t offset, const T &first, const Rests &... rest);


class XPlaneUdp {
    public:
        struct DatarefIndex {
            const size_t idx;
        };
        struct PlaneInfo {
            double lon, lat, alt; // 经纬度 高度
            float agl, pitch, track, roll; // 离地高 / 俯仰 真航向 滚转
            float vX, vY, vZ, rollRate, pitchRate, yawRate; // 三轴速度 / 横滚 俯仰 偏航
        };

        XPlaneUdp ();
        XPlaneUdp (const XPlaneUdp &) = delete;
        XPlaneUdp& operator= (const XPlaneUdp &) = delete;
        XPlaneUdp (XPlaneUdp &&) = delete;
        XPlaneUdp& operator= (XPlaneUdp &&) = delete;

        DatarefIndex addDataref (const std::string &dataref, int32_t freq = 1, int index = -1);
        DatarefIndex addDatarefArray (const std::string &dataref, int length, int32_t freq = 1);
        bool getDataref (const DatarefIndex &dataref, float &value, float defaultValue = 0) const;
        template <Container T>
        bool getDataref (const DatarefIndex &dataref, T &container, float defaultValue = 0);
        void changeDatarefFreq (const DatarefIndex &dataref, float freq);
        void setDataref (const std::string &dataref, float value, int index = -1);
        template <Container T>
        void setDatarefArray (const std::string &dataref, const T &value);

        void addPlaneInfo (int freq = 1);
        void getPlaneInfo (PlaneInfo &infoDst) const;
    private:
        struct DatarefInfo {
            std::string name; // dataref 长度
            int start, end; // [start,end]
            int32_t freq; // 频率
            bool available; // 是否可用
            bool isArray; // 是否是数组
        };

        // 数据
        std::vector<DatarefInfo> dataRefs;
        std::vector<float> values;
        boost::dynamic_bitset<> space;
        std::unordered_map<std::string, size_t> exist;
        PlaneInfo info{.track = -1};
        // 网络
        asio::io_context io_context{}; // 上下文
        asio::executor_work_guard<asio::io_context::executor_type> workGuard;
        ip::udp::socket multicastSocket{io_context}; // 监听多播
        ip::udp::socket xpSocket{io_context}; // xp通信
        ip::udp::endpoint xpEndpoint; // xp端口

        void detect ();
        size_t findSpace (size_t length);
        void extendSpace ();
        void sendData (std::vector<char> data);
        asio::awaitable<void> send (std::vector<char> &&data);
        void receiveData();
        asio::awaitable<void> receive ();
        void receiveDataProcess(std::vector<char> data);
};

/**
 * @brief 解包一串字符数据
 * @param container 容器
 * @param offset 偏移字节
 */
template <typename CharContainer, typename First, typename... Rests>
void unpack (const CharContainer &container, size_t offset, First &first, Rests &... rest) {
    memcpy(&first, container.data() + offset, sizeof(First));
    if constexpr (sizeof...(rest) > 0)
        unpack(container, offset + sizeof(First), rest...);
}

/**
 * @brief 打包为字符数组
 * @param container 容器
 * @param offset 偏移量
 * @param first string,基本类型
 * @return 打包数据量
 */
template <typename T, typename... Rests>
    requires (std::same_as<std::string, T> || std::is_fundamental_v<T>)
size_t pack (std::vector<char> &container, const size_t offset, const T &first, const Rests &... rest) {
    if constexpr (std::same_as<std::string, T>) { // string
        memcpy(container.data() + offset, first.data(), first.size());
        if constexpr (sizeof...(rest) > 0)
            return pack(container, offset + first.size(), rest...);
        else
            return offset + first.size();
    } else { // 基本类型
        memcpy(container.data() + offset, &first, sizeof(T));
        if constexpr (sizeof...(rest) > 0)
            return pack(container, offset + sizeof(T), rest...);
        else
            return offset + sizeof(T);
    }
}

/**
 * @brief 获取 dataref 最新值
 * @param dataref 标识
 * @param container 容器
 * @param defaultValue 默认值
 * @return 值可用
 */
template <Container T>
bool XPlaneUdp::getDataref (const DatarefIndex &dataref, T &container, float defaultValue) {
    const auto &ref = dataRefs[dataref.idx];
    const size_t size = ref.end - ref.start + 1;
    if (!ref.available) {
        std::ranges::fill(container | std::views::take(size), defaultValue);
        return false;
    }
    auto source = values | std::views::drop(ref.start) | std::views::take(std::min(size, container.size()));
    std::ranges::copy(source, container.begin());
    return true;
}

/**
 * @brief 设置某组 dataref 值
 * @param dataref dataref 名称
 * @param value 容器
 */
template <Container T>
void XPlaneUdp::setDatarefArray (const std::string &dataref, const T &value) {
    std::vector<char> buffer(509);
    for (int i = 0; i < value.size(); ++i) {
        pack(buffer, 0, DATAREF_SET_HEAD, value[i], std::format("{}[{}]", dataref, i), '\x00');
        sendData(buffer);
    }
}

#endif
