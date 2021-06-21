#ifndef OBJECT_DETECTION_MSGS_SORT_HPP
#define OBJECT_DETECTION_MSGS_SORT_HPP

#include <algorithm>
#include <vector>

#include <object_detection_msgs/Objects.h>

namespace object_detection_msgs {

// ***************************************
// Helpers for sorting multiple containers
// ***************************************

// 
template < class Iterator, class Compare > struct IndexCompare {
  IndexCompare(std::vector< Iterator > &_itors, Compare &_comp) : itors(_itors), comp(_comp) {}

  bool operator()(const std::size_t a, const std::size_t b) const {
    return comp(*itors[a], *itors[b]);
  }

  std::vector< Iterator > &itors;
  Compare &comp;
};

// permutation of the given iterator range.
// the value of perms[i] indicates the rank of the i-th element in the range.
template < class Iterator, class Compare >
std::vector< std::size_t > permutation(Iterator first, Iterator last, Compare comp) {
  std::vector< Iterator > itors;
  std::vector< std::size_t > perms;
  for (Iterator itor = first; itor != last; ++itor) {
    itors.push_back(itor);
    perms.push_back(perms.size());
  }
  std::sort(perms.begin(), perms.end(), IndexCompare< Iterator, Compare >(itors, comp));
  return perms;
}

// swap operations to sort the given permutation.
// the value of ops[i] indicates swapping of i-th and perms[i] elements.
std::vector< std::size_t > operationsToSort(std::vector< std::size_t > perms) {
  std::vector< std::size_t > ops;
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
template < class Iterator >
bool applyOperationsToSort(Iterator first, Iterator last, const std::vector< std::size_t > &ops) {
  std::vector< Iterator > itors;
  for (Iterator itor = first; itor != last; ++itor) {
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

template < class Compare > bool sortByName(Objects *const objs, Compare comp) {
  if (!objs) {
    return false;
  }
  const std::vector< std::size_t > ops(
      operationsToSort(permutation(objs->names.begin(), objs->names.end(), comp)));
  return applyOperationsToSort(objs->names.begin(), objs->names.end(), ops) &&
         applyOperationsToSort(objs->probabilities.begin(), objs->probabilities.end(), ops) &&
         applyOperationsToSort(objs->contours.begin(), objs->contours.end(), ops);
}

template < class Compare > bool sortByProbability(Objects *const objs, Compare comp) {
  if (!objs) {
    return false;
  }
  const std::vector< std::size_t > ops(
      operationsToSort(permutation(objs->probabilities.begin(), objs->probabilities.end(), comp)));
  return applyOperationsToSort(objs->names.begin(), objs->names.end(), ops) &&
         applyOperationsToSort(objs->probabilities.begin(), objs->probabilities.end(), ops) &&
         applyOperationsToSort(objs->contours.begin(), objs->contours.end(), ops);
}

template < class Compare > bool sortByContour(Objects *const objs, Compare comp) {
  if (!objs) {
    return false;
  }
  const std::vector< std::size_t > ops(
      operationsToSort(permutation(objs->contours.begin(), objs->contours.end(), comp)));
  return applyOperationsToSort(objs->names.begin(), objs->names.end(), ops) &&
         applyOperationsToSort(objs->probabilities.begin(), objs->probabilities.end(), ops) &&
         applyOperationsToSort(objs->contours.begin(), objs->contours.end(), ops);
}

} // namespace object_detection_msgs

#endif