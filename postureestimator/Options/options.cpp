/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>
#include <sstream>
#include <list>

#include "options.h"

OptionError::OptionError(const std::string& message, const std::string& help)
: message(message), help(help) {}

CommonOptions::CommonOptions(char **argv, int argc,
                             std::list<std::shared_ptr<AppOption<int>>>& int_options,
                             std::list<std::shared_ptr<AppOption<std::string>>>& string_options)
{
    BOOST_LOG_TRIVIAL(trace) << "Parsing command line options.";
    program_name = argv[0];

    po::options_description desc("Common options");
    desc.add_options()
        ("help", "produce help message")
        ("verbose", "be really verbose (you have been warned)");

    po::options_description app_specific("Applicationspecific options.");
    for (std::shared_ptr<AppOption<int>>& o : int_options)
        app_specific.add(o->description);
    for (std::shared_ptr<AppOption<std::string>>& o : string_options)
        app_specific.add(o->description);
    desc.add(app_specific);

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    unrecognized_options = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    vm.notify();

    std::stringstream ss;
    desc.print(ss);
    const std::string help_text = ss.str();

    if (vm.count("help"))
        throw help_text;
    verbose = vm.count("verbose");

    try {
        for (std::shared_ptr<AppOption<int>>& o : int_options)
            o->set_value(vm);
        for (std::shared_ptr<AppOption<std::string>>& o : string_options)
            o->set_value(vm);
    } catch (const std::string& message) {
        throw OptionError(message, help_text);
    }
}
