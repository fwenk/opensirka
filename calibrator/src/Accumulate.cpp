/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
#include "Accumulate.h"

/**
 * Compute the difference accumulates of all adjacent accumulates in 'accumulates'.
 * That is: deltas[k] = accumulates[k]^{-1} * accumulates[k+1]
 */
void compute_delta_accumulate_run(AccumulateRun& deltas, const AccumulateRun& accumulates)
{
    assert (accumulates.size() >= 2);
    AccumulateRun::const_iterator header = accumulates.begin();
    AccumulateRun::const_iterator trailer = header++;
    const AccumulateRun::const_iterator end = accumulates.end();

    deltas.clear();
    for (; header != end; ++header, ++trailer)
        deltas.push_back(*trailer % *header);
}
