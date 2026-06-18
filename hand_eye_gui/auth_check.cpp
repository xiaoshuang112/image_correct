#include "auth_check.hpp"

#include <QSettings>
#include <QFile>
#include <QDir>
#include <QDebug>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

std::string check_authorization(const std::string& appPath)
{
    // 1. 检查 cfg.ini
    std::string cfg_path = appPath + "cfg.ini";
    if (!fs::exists(cfg_path)) {
        return "缺少配置文件 cfg.ini\n路径: " + cfg_path;
    }

    // 2. 读取 model_name
    QSettings settings(QString::fromStdString(cfg_path), QSettings::IniFormat);
    QString md_name = settings.value("cfg/model_name", "").toString();
    if (md_name.isEmpty()) {
        return "cfg.ini 中未配置 model_name\n请在 [cfg] 节下添加 model_name=xxx";
    }

    qDebug() << "[auth] model_name:" << md_name;

    // 3. 检查 models/<model_name>.m
    std::string model_path = appPath + "models/" + md_name.toStdString() + ".m";
    if (!fs::exists(model_path)) {
        return "缺少模型文件: " + model_path;
    }

    // 4. 读取 .m 文件
    std::ifstream ifs(model_path, std::ios::binary);
    if (!ifs.is_open()) {
        return "无法打开模型文件: " + model_path;
    }
    ifs.seekg(0, std::ios::end);
    size_t modelSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::unique_ptr<char[]> engineData(new char[modelSize]);
    ifs.read(engineData.get(), modelSize);
    ifs.close();

    // 5. 判断是否加密模型
    char* pData = engineData.get();
    int size_bytes = sizeof(unsigned long long);
    unsigned long long data_size = *((unsigned long long*)pData);

    if (modelSize == (data_size + size_bytes)) {
        // ---- 这是加密模型，需要验证授权 ----
        qDebug() << "[auth] 检测到加密模型，验证设备授权...";

        std::vector<std::string> mac_list = get_mac_address_list();
        qDebug() << "[auth] 本机 MAC 地址:" << mac_list.size() << "个";

        std::vector<char> data_buf;
        std::string encode_key = md_name.toStdString();
        for (auto& mac : mac_list) {
            int ret = decode_buf(engineData.get(), modelSize, mac, encode_key, data_buf);
            if (0 == ret) {
                qDebug() << "[auth] 授权验证通过 (MAC:" << mac.c_str() << ")";
                break;
            }
        }

        if (data_buf.empty()) {
            return "本设备未授权使用本程序\n\n"
                   "模型文件: " + model_path + "\n"
                   "原因: 设备 MAC 地址未在授权列表中";
        }
    } else {
        // ---- 普通模型（未加密）
        return "本设备未授权使用本程序\n\n";
    }

    return "";  // 空字符串 = 通过
}
