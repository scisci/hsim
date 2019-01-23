/*! \file RealRangeDichotomy.hpp
    \brief Continuous range iteration.
 
    RealRangeDichotomy allows you to iterate a range of values using a binary
    search methodology. However it works on continuous ranges, rather than
    discrete. So you give it a range like 0.0 - 1.0 and a number of iterations
    and it will iterate: 0.5 | 0.25, 0.75 | 0.125, 0.375, 0.625, 0.875 | etc
    It makes smaller and smaller increments and eventually approaches the edges
    of the range.
*/

#ifndef HSIM_REAL_RANGE_DICHOTOMY_H
#define HSIM_REAL_RANGE_DICHOTOMY_H

#include "hsim/Math.hpp"

namespace hsim {

class RealRangeDichotomy;

class RealRangeDichotomyIterator {
public:
  RealRangeDichotomyIterator(const RealRangeDichotomy& range);
  RealRangeDichotomyIterator(const RealRangeDichotomy& range, std::size_t max_steps);
  
  bool operator!=(const RealRangeDichotomyIterator& other);

  const RealRangeDichotomyIterator& operator++();

  const Real& operator*() const;
  
private:
  Real value_;
  std::size_t level_;
  std::size_t level_step_;
  
  std::size_t num_level_steps_;
  const RealRangeDichotomy& range_;
};

class RealRangeDichotomy {
public:
  RealRangeDichotomy(Real min, Real max, size_t max_steps)
  : min_(min), max_(max), max_steps_(max_steps) {}
  
  // Required
  RealRangeDichotomyIterator begin() const
  {
      return RealRangeDichotomyIterator(*this);
  }

  // Required
  RealRangeDichotomyIterator end() const
  {
      return RealRangeDichotomyIterator(*this, max_steps_);
  }
  
private:
  friend class RealRangeDichotomyIterator;
  
  Real min_;
  Real max_;
  std::size_t max_steps_;
};

} // namespace hsim

#endif /* HSIM_REAL_RANGE_DICHOTOMY_H */
