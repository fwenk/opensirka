/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#define BOOST_TEST_MODULE "test_loadaccumulate"

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>

#include "../src/IMUAccumulate.h"

namespace bf = boost::filesystem;

/** String per-line feedable from an input stream. */
class Line : public std::string
{
    friend std::istream& operator>>(std::istream& is, Line& line)
    {
        return std::getline(is, line);
    }
};

/** Count the non-commentary lines in an accumulate file.
 * That is, iterate over all lines and increment a counter
 * for each line not starting with '#'. */
static unsigned count_data_lines_in_file(const std::string& filename)
{
    using namespace std;
    ifstream ifs(filename);
    BOOST_REQUIRE(ifs.is_open());

    typedef istream_iterator<Line> LineIterator;
    return count_if(LineIterator(ifs), LineIterator(), [](const Line& l) -> bool {
        return l[0] != '#';
    });
}

BOOST_AUTO_TEST_CASE(load_accumulates)
{
    BOOST_LOG_TRIVIAL(info) << "Loading accumulate series.";

    /* Get the path of the test data file. */
    if (boost::unit_test::framework::master_test_suite().argc < 2)
        BOOST_FAIL("Path of test data file not given.");
    const char *filename = boost::unit_test::framework::master_test_suite().argv[1];
    BOOST_LOG_TRIVIAL(info) << "Loading test data file: " << filename;

    /* Load the accumulates. */
    LIR::IMUAccumulates accumulates(filename);
    const unsigned Ndatalines = count_data_lines_in_file(filename);
    const unsigned Nentries = accumulates.entries.size();

    BOOST_CHECK_EQUAL(Nentries, Ndatalines);
}
