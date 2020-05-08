//
// Created by kevin on 2020/5/7.
//

#ifndef BI2PC_CONFIG_H
#define BI2PC_CONFIG_H
#include "bino/common_inc.h"
namespace kbino{

/**
 * 配置类，使用SetParameterFile确定配置文件
 * 然后用Get得到对应值
 * 单例模式
 */
    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {}  // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        // access the parameter values
        template <typename T>
        static T Get(const std::string &key) {
            return T(Config::config_->file_[key]);
        }
    };
}
#endif //BI2PC_CONFIG_H
