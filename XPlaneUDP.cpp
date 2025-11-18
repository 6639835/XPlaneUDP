#include "XPlaneUDP.hpp"

#ifdef _WIN32
constexpr bool IS_WIN = true;
#else
constexpr bool IS_WIN = false;
#endif

static constexpr std::string MULTI_CAST_GROUP{"239.255.1.1"};
static constexpr unsigned short MULTI_CAST_PORT{49707};


XPlaneUdp::XPlaneUdp () : workGuard(asio::make_work_guard(io_context)) {}

/**
 * @brief 新增监听目标
 * @param dataref dataref 名称
 * @param freq 频率
 * @param index 目标为数组时的索引
 */
XPlaneUdp::DatarefIndex XPlaneUdp::addDataref (const std::string &dataref, int32_t freq, int index) {
    std::string name = (index == -1) ? dataref : std::format("{}[{}]", dataref, index);
    if (const auto it = exist.find(name); it != exist.end()) {
        std::cerr << "already exist! nothing change.";
        return {it->second};
    }
    size_t start = findSpace(1);
    dataRefs.emplace_back(name, start, start, freq, true, false);
    std::vector<char> buffer(413);
    pack(buffer, 0, DATAREF_GET_HEAD, freq, start, name);
    sendData(std::move(buffer));
    exist[name] = dataref.size() - 1;
    return {dataRefs.size() - 1};
}

/**
 * @brief 新增监听目标,目标为数组
 * @param dataref dataref 名称
 * @param length 数组长度
 * @param freq 频率
 */
XPlaneUdp::DatarefIndex XPlaneUdp::addDatarefArray (const std::string &dataref, const int length, int32_t freq) {
    if (const auto it = exist.find(dataref); it != exist.end()) {
        std::cerr << "already exist! nothing change.";
        return {it->second};
    }
    int start = static_cast<int>(findSpace(length));
    dataRefs.emplace_back(dataref, start, start + length - 1, freq, true, true);
    for (int i = 0; i < length; ++i) {
        std::string name = std::format("{}[{}]", dataref, i);
        std::vector<char> buffer(413);
        pack(buffer, 0, DATAREF_GET_HEAD, freq, start + i, name);
        sendData(std::move(buffer));
    }
    exist[dataref] = dataRefs.size() - 1;
    return {dataRefs.size() - 1};
}

/**
 * @brief 获取 dataref 最新值
 * @param dataref 标识
 * @param value 返回值
 * @param defaultValue 默认值
 * @return 值可用
 */
bool XPlaneUdp::getDataref (const DatarefIndex &dataref, float &value, const float defaultValue) const {
    if (!dataRefs[dataref.idx].available) {
        value = defaultValue;
        return false;
    }
    value = values[dataref.idx];
    return true;
}

/**
 * @brief 修改获取 dataref 的频率
 * @param dataref 标识
 * @param freq 频率
 */
void XPlaneUdp::changeDatarefFreq (const DatarefIndex &dataref, const float freq) {
    auto &ref = dataRefs[dataref.idx];
    const int size = ref.end - ref.start + 1;
    if (freq == 0) { // 停止接收
        if (!ref.available)
            return;
        ref.available = false;
        space.set(ref.start, size, false);
    } else {
        // 先恢复
        if (!ref.available) {
            ref.available = true;
            const int start = static_cast<int>(findSpace(size));
            ref.start = start;
            ref.end = start + size - 1;
        }
        // 再发送
        std::vector<char> buffer(413);
        if (!ref.isArray) {
            pack(buffer, 0, DATAREF_GET_HEAD, freq, ref.start, ref.name);
            sendData(std::move(buffer));
        } else {
            for (size_t i = 0; i < size; ++i) {
                pack(buffer, 0, DATAREF_GET_HEAD, freq, ref.start + i, std::format("{}[{}]", ref.name, i));
                sendData(buffer);
            }
        }
    }
}

/**
 * @brief 设置dataref值
 * @param dataref dataref 名称
 * @param value 值
 * @param index 目标为数组时的索引
 */
void XPlaneUdp::setDataref (const std::string &dataref, const float value, int index) {
    std::vector<char> buffer(509);
    const std::string name = (index == -1) ? dataref : std::format("{}[{}]", dataref, index);
    pack(buffer, 0, DATAREF_SET_HEAD, value, name, '\x00');
    sendData(std::move(buffer));
}

/**
 * @brief 开始接收基本信息
 * @param freq 接收频率
 */
void XPlaneUdp::addPlaneInfo (int freq) {
    const std::string sentence = std::format("{}{}\x00", BASIC_INFO_HEAD, freq);
    std::vector<char> buffer(sentence.size());
    pack(buffer, 0, sentence);
    sendData(std::move(buffer));
}

/**
 * @brief 获取基本信息最新值
 */
void XPlaneUdp::getPlaneInfo (PlaneInfo &infoDst) const {
    infoDst = info;
}

/**
 * @brief 监听XPlane是否在线
 */
void XPlaneUdp::detect () {
    // 绑定到多播
    multicastSocket.open(ip::udp::v4());
    const asio::socket_base::reuse_address option(true);
    multicastSocket.set_option(option);
    ip::udp::endpoint multicastEndpoint;
    if (IS_WIN)
        multicastEndpoint = ip::udp::endpoint(ip::udp::v4(), MULTI_CAST_PORT);
    else
        multicastEndpoint = ip::udp::endpoint(ip::make_address(MULTI_CAST_GROUP), MULTI_CAST_PORT);
    multicastSocket.bind(multicastEndpoint);
    const ip::address_v4 multicast_address = ip::make_address_v4(MULTI_CAST_GROUP);
    multicastSocket.set_option(ip::multicast::join_group(multicast_address));
}

/**
 * @brief 找到一段连续可用的空间
 * @param length 长度
 * @return 初始位置
 */
size_t XPlaneUdp::findSpace (const size_t length) {
    size_t start{}, count{};
    for (size_t i = 0; i < space.size(); ++i) {
        if (!space[i]) {
            if (count == 0)
                start = i;
            ++count;
            if (count >= length) {
                space.set(start, start + length, true);
                extendSpace();
                return start;
            }
        } else {
            count = 0;
        }
    }
    // 补充
    for (int i = 0; i < length; ++i)
        space.push_back(true);
    extendSpace();
    return space.size() - length;
}

void XPlaneUdp::extendSpace () {
    for (int i = 0; i < (space.size() - values.size()); ++i)
        values.emplace_back();
}

/**
 * @brief 向xp发送udp数据
 * @param data 数据
 */
void XPlaneUdp::sendData (std::vector<char> data) {
    asio::co_spawn(io_context, send(std::move(data)), asio::detached);
}

asio::awaitable<void> XPlaneUdp::send (std::vector<char> &&data) {
    const auto buffer = std::make_shared<std::vector<char>>(std::move(data));
    co_await xpSocket.async_send_to(asio::buffer(*buffer), xpEndpoint, asio::use_awaitable);
}

/**
 * @brief 接收数据
 */
void XPlaneUdp::receiveData () {
    asio::co_spawn(io_context, receive(), asio::detached);
}

asio::awaitable<void> XPlaneUdp::receive () {
    ip::udp::endpoint xpSend;
    std::array<char, 1472> receiveBuffer{};
    while (xpSocket.is_open()) {
        // 这里非常非常怪 应该是 std::tuple<boost::system::error_code, unsigned long long>
        size_t bytes_received = co_await xpSocket.async_receive_from(
            asio::buffer(receiveBuffer), xpSend, asio::use_awaitable);
        receiveDataProcess({receiveBuffer.begin(), receiveBuffer.begin() + bytes_received});
    }
}

void XPlaneUdp::receiveDataProcess (std::vector<char> data) {
    if (equal(DATAREF_GET_HEAD.begin(), DATAREF_GET_HEAD.begin() + 4, data.begin())) { // dataref,文档有误实际返回 RREF,
        if ((data.size() - 5) % 8 != 0)
            return;
        for (int i = HEADER_LENGTH; i < data.size(); i += 8) {
            int index;
            float value;
            unpack(data, i, index, value);
            values[index] = value;
        }
    } else if (equal(BASIC_INFO_HEAD.begin(), BASIC_INFO_HEAD.begin() + 4, data.begin())) { // 基本信息
        if (((data.size() - 5) % 64 != 0) || (data.size() <= 6))
            return;
        unpack(data, HEADER_LENGTH, info);
    }
}
