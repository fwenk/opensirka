/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __OPTIONS_H_
#define __OPTIONS_H_

#include <list>
#include <vector>
#include <string>
#include <boost/program_options.hpp>
#include <sstream>

namespace po = boost::program_options;

template <typename T>
class AppOption
{
public:
    const std::string name;
    boost::shared_ptr<po::option_description> description;
    bool required;
    bool set;
    T value;

    AppOption(bool required, const std::string& name, const std::string& text_description)
    : name(name), description(new po::option_description(name.c_str(), po::value<T>(), text_description.c_str())), required(required), set(false) {}

    void set_value(po::variables_map& vm) {
        if (vm.count(name)) {
            value = vm[name].template as<T>();
            set = true;
        } else if (required) {
            std::stringstream ss;
            ss << "Option --" << name << " missing.";
            throw ss.str();
        }
    }
};
typedef AppOption<int> IntOption;
typedef AppOption<std::string> StringOption;
typedef AppOption<bool> BoolOption;
typedef std::shared_ptr<IntOption> SpIntOption;
typedef std::shared_ptr<StringOption> SpStringOption;
typedef std::shared_ptr<BoolOption> SpBoolOption;

class OptionError {
public:
    const std::string message;
    const std::string help;
    OptionError(const std::string& message, const std::string& help);
};


class CommonOptions {
public:
    std::string program_name;
    bool verbose;
    std::vector<std::string> unrecognized_options;

    CommonOptions(char **argv, int argc,
                  std::list<std::shared_ptr<AppOption<int>>>& int_options,
                  std::list<std::shared_ptr<AppOption<std::string>>>& string_options);
};

#endif // __OPTIONS_H_
