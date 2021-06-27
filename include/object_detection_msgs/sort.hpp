#ifndef OBJECT_DETECTION_MSGS_SORT_HPP
#define OBJECT_DETECTION_MSGS_SORT_HPP

#include <algorithm>
#include <vector>

#include <boost/range/algorithm/sort.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/end.hpp>
#include <boost/range/iterator.hpp>

#include <object_detection_msgs/Objects.h>

namespace object_detection_msgs {

// ***************************************
// Helpers for sorting multiple containers
// ***************************************

// permutation of the given iterator range.
// the value of perms[i] indicates the rank of the i-th element in the range.
template <class Range, class Compare>
std::vector<std::size_t> permutation(const Range &range, Compare comp) {
  using Iterator = typename boost::range_iterator<const Range>::type;
  std::vector<Iterator> itors;
  std::vector<std::size_t> perms;
  for (Iterator itor = boost::begin(range); itor != boost::end(range); ++itor) {
    itors.push_back(itor);
    perms.push_back(perms.size());
  }
  boost::sort(perms, [&comp, &itors](const std::size_t a, const std::size_t b) {
    return comp(*itors[a], *itors[b]);
  });
  return perms;
}

// swap operations to sort the given permutation.
// the value of ops[i] indicates swapping of i-th and perms[i] elements.
std::vector<std::size_t> operationsToSort(std::vector<std::size_t> perms) {
  std::vector<std::size_t> ops;
  for (std::size_t i = 0; i < perms.size(); ++i) {
    ops.push_back(perms[i]);
    for (std::size_t j = i + 1; j < perms.size(); ++j) {
      if (perms[j] == i) {
        std::swap(perms[i], perms[j]);
        break;
      }
    }
  }
  return ops;
}

// apply the given swap operations to the given range
template <class Range>
bool applyOperationsToSort(Range *const range, const std::vector<std::size_t> &ops) {
  using Iterator = typename boost::range_iterator<Range>::type;
  std::vector<Iterator> itors;
  for (Iterator itor = boost::begin(*range); itor != boost::end(*range); ++itor) {
    itors.push_back(itor);
  }

  for (std::size_t i = 0; i < ops.size() && i < itors.size(); ++i) {
    const std::size_t j = ops[i];
    if (j < 0 || j >= itors.size()) {
      return false;
    }
    if (j != i) {
      std::swap(*itors[i], *itors[j]);
    }
  }

  return true;
}

// **************************
// Sort functions for Objects
// **************************

template <class Compare> bool sortByName(Objects *const objs, Compare comp) {
  if (!objs) {
    return false;
  }
  const std::vector<std::size_t> ops = operationsToSort(permutation(objs->names, comp));
  return applyOperationsToSort(&objs->names, ops) &&
         applyOperationsToSort(&objs->probabilities, ops) &&
         applyOperationsToSort(&objs->contours, ops);
}

template <class Compare> bool sortByProbability(Objects *const objs, Compare comp) {
  if (!objs) {
    return false;
  }
  const std::vector<std::size_t> ops = operationsToSort(permutation(objs->probabilities, comp));
  return applyOperationsToSort(&objs->names, ops) &&
         applyOperationsToSort(&objs->probabilities, ops) &&
         applyOperationsToSort(&objs->contours, ops);
}

template <class Compare> bool sortByContour(Objects *const objs, Compare comp) {
  if (!objs) {
    return false;
  }
  const std::vector<std::size_t> ops = operationsToSort(permutation(objs->contours, comp));
  return applyOperationsToSort(&objs->names, ops) &&
         applyOperationsToSort(&objs->probabilities, ops) &&
         applyOperationsToSort(&objs->contours, ops);
}

} // namespace object_detection_msgs

#endif