//
//  RealRangeDichotomy.cpp
//  hsim
//
//  Created by z on 1/22/19.
//

#include "hsim/RealRangeDichotomy.hpp"

namespace hsim {


RealRangeDichotomyIterator::RealRangeDichotomyIterator(
  const RealRangeDichotomy& range)
  : range_(range),
    level_(0),
    level_step_(0),
    num_level_steps_(1),
    value_((range.min_ + range.max_) / 2.0)
{}

RealRangeDichotomyIterator::RealRangeDichotomyIterator(
  const RealRangeDichotomy& range, std::size_t max_steps)
  : range_(range),
    level_(log2(max_steps + 1)),
    level_step_(0),
    num_level_steps_(0),
    value_(0.0)
{}
  
bool RealRangeDichotomyIterator::operator!=(const RealRangeDichotomyIterator& other)
{
  return level_ != other.level_ || level_step_ != other.level_step_;
}

const RealRangeDichotomyIterator& RealRangeDichotomyIterator::operator++()
{
  if (++level_step_ == num_level_steps_) {
    level_++;
    level_step_ = 0;
    num_level_steps_ *= 2;
  }
  
  const Real dif = range_.max_ - range_.min_;
  const Real stride = dif / num_level_steps_;
  const Real start = range_.min_ + stride / 2.0;
  value_ = start + stride * level_step_;
  return *this;
}

const Real& RealRangeDichotomyIterator::operator*() const
{
  return value_;
}


} // namespace hsim
