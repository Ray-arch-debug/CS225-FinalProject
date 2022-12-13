
/**
 * @file heap.cpp
 * Implementation of a heap class. (Written in lab_heap)
 */
#include <cstddef>
#include "heap.h"


// heap is stored starting at index 1

// template <class T, class Compare>
size_t heap::root() const
{
    // @TODO Update to return the index you are choosing to be your root.
    return 1;
}

// template <class T, class Compare>
size_t heap::leftChild(size_t currentIdx) const
{
    // @TODO Update to return the index of the left child.
    return (2 * currentIdx);
}

// template <class T, class Compare>
size_t heap::rightChild(size_t currentIdx) const
{
    // @TODO Update to return the index of the right child.
    return (2 * currentIdx) + 1;
}

// template <class T, class Compare>
size_t heap::parent(size_t currentIdx) const
{
    // @TODO Update to return the index of the parent.
    return (currentIdx / 2); // need to include floor or not?
}

// template <class T, class Compare>
bool heap::hasAChild(size_t currentIdx) const
{
    // @TODO Update to return whether the given node has a child
    size_t leftChildIdx = leftChild(currentIdx);
    if (leftChildIdx >= _elems.size()) {
        return false;
    }
    // if reach here, we know the left child exists
    return true;
}

// template <class T, class Compare>
size_t heap::maxPriorityChild(size_t currentIdx) const
{
    // @TODO Update to return the index of the child with highest priority
    ///   as defined by higherPriority()

    // we can assume that at least one child exists (i.e, at least the left child exists)

    size_t leftChildIdx = leftChild(currentIdx);
    size_t rightChildIdx = rightChild(currentIdx);

    // check if rightchild exists
    if (rightChildIdx >= _elems.size()) {
        // right child does not exist
        return leftChildIdx;
    }

    // both children exist, so check which one has higher priority
    if (_elems[leftChildIdx].second < _elems[rightChildIdx].second) {
        return leftChildIdx;
    }
    /*if (higherPriority(_elems[leftChildIdx], _elems[rightChildIdx])) {
        return leftChildIdx;
    }*/

    // if reach here, then right child has higher priority
    return rightChildIdx;
}

// template <class T, class Compare>
void heap::heapifyDown(size_t currentIdx)
{
    // @TODO Implement the heapifyDown algorithm.]

    // check if current index has a child
    if (!hasAChild(currentIdx)) {
        return;
    }

    // if reach here, the current index has (at least one) a child
    size_t maxPriorityChildIdx = maxPriorityChild(currentIdx);

    // check if child index has higher priority than current 
    if (_elems[maxPriorityChildIdx].second < _elems[currentIdx].second) {
        // update positions 
        /*if (positions_.find(_elems[maxPriorityChildIdx].first) != positions_.end() && positions_.find(_elems[currentIdx].first) != positions_.end()) {
            positions_.at(_elems[maxPriorityChildIdx].first) = currentIdx;
            positions_.at(_elems[currentIdx].first) = maxPriorityChildIdx;
        } else {
            positions_.insert({_elems[maxPriorityChildIdx].first, currentIdx});
            positions_.insert({_elems[currentIdx].first, maxPriorityChildIdx});
        }*/

        positions_[_elems[maxPriorityChildIdx].first] = currentIdx;
        positions_[_elems[currentIdx].first] = maxPriorityChildIdx;

        std::swap(_elems[currentIdx], _elems[maxPriorityChildIdx]);
        heapifyDown(maxPriorityChildIdx); // continue heapifying down
    } else {
        return;
    }

    /*
    if (higherPriority(_elems[maxPriorityChildIdx], _elems[currentIdx])) {
        std::swap(_elems[currentIdx], _elems[maxPriorityChildIdx]);
        heapifyDown(maxPriorityChildIdx); // continue heapifying down
    } else {
        return;
    }
    */
}

// template <class T, class Compare>
void heap::heapifyUp(size_t currentIdx)
{
    if (currentIdx == root())
        return;
    size_t parentIdx = parent(currentIdx);

    if (_elems[currentIdx].second < _elems[parentIdx].second) {
        /*if (positions_.find(_elems[currentIdx].first) != positions_.end() && positions_.find(_elems[parentIdx].first) != positions_.end()) {
            positions_[_elems[currentIdx].first] = parentIdx;
            positions_[_elems[parentIdx].first] = currentIdx;
        } else {
            positions_.insert({_elems[currentIdx].first, parentIdx});
            positions_.insert({_elems[parentIdx].first, currentIdx});
        }*/
        // update positions 
        //std::cout << "here" << std::endl;
        //std::cout << positions_[_elems[currentIdx].first] << std::endl;
        //std::cout << positions_[_elems[parentIdx].first] << std::endl;
        //std::cout << std::endl;
        positions_[_elems[currentIdx].first] = parentIdx;
        positions_[_elems[parentIdx].first] = currentIdx;

        std::swap(_elems[currentIdx], _elems[parentIdx]);
        heapifyUp(parentIdx);
    }

    /*
    if (higherPriority(_elems[currentIdx], _elems[parentIdx])) {
        std::swap(_elems[currentIdx], _elems[parentIdx]);
        heapifyUp(parentIdx);
    }
    */
}

// template <class T, class Compare>
heap::heap()
{
    // @TODO Depending on your implementation, this function may or may
    ///   not need modifying
}

// template <class T, class Compare>
heap::heap(const std::vector<std::pair<int, double>>& elems) 
{
    // @TODO Construct a heap using the buildHeap algorithm
    std::pair<int, double> placeholder;
    _elems.push_back(placeholder); // create extra element at index 0

    // copy over elements
    for (size_t index = 0; index < elems.size(); ++index) {
        positions_[elems[index].first] = root() + index; // important
        _elems.push_back(elems[index]);
    }

    
    size_t lastElemParentIdx = parent(_elems.size() - 1);
    for (size_t index = lastElemParentIdx; index >= root(); --index) {
        heapifyDown(index);
    }
}

// template <class T, class Compare>
std::pair<int, double> heap::pop()
{
    // @TODO Remove, and return, the element with highest priority

    std::pair<int, double> highest_priority = peek(); // retrieve element with highest priority

    // update positions 
    positions_[_elems[root()].first] = BigNum;
    positions_[_elems[_elems.size() - 1].first] = root();

    std::swap(_elems[root()], _elems[_elems.size() - 1]); // swap root (highest priority) with last element

    _elems.pop_back(); // remove last element (which was originally the root node)

    if (!empty()) {
        heapifyDown(root());
    }

    if (_elems.size() == 1) {
        _elems.pop_back();
    }
    
    return highest_priority;
}

// template <class T, class Compare>
std::pair<int, double> heap::peek() const
{
    // @TODO Return, but do not remove, the element with highest priority
    if (!empty()) {
        return _elems[root()];
    }
    std::pair<int, double> empty;
    return empty;
}

// template <class T, class Compare>
void heap::push(const std::pair<int, double>& elem)
{
    // @TODO Add elem to the heap
    if (empty()) {
        std::pair<int, double> placeholder;
        _elems.push_back(placeholder);
    }
    _elems.push_back(elem); // elem is now at index 1, the root index
    positions_[elem.first] = _elems.size() - 1;
    heapifyUp(_elems.size() - 1); // call heapifyUp on last index of _elems
}

// template <class T, class Compare>
void heap::updateElem(const size_t & idx, const std::pair<int, double>& elem)
{
    // @TODO In-place updates the value stored in the heap array at idx
    // Corrects the heap to remain as a valid heap even after update
    if (idx >= _elems.size()) {
        return;
    }
    _elems[idx] = elem;

    positions_[elem.first] = idx; // important

    size_t parentIdx = parent(idx);

    if (_elems[parentIdx].second < _elems[idx].second) {
        heapifyDown(idx);
    } else {
        heapifyUp(idx);
    }

    /*
    if (higherPriority(_elems[parentIdx], _elems[idx])) {
        heapifyDown(idx);
    } else {
        heapifyUp(idx);
    }
    */
}


// template <class T, class Compare>
bool heap::empty() const
{
    // @TODO Determine if the heap is empty
    return _elems.empty();
}

// template <class T, class Compare>
void heap::getElems(std::vector<std::pair<int, double>> & heaped) const
{
    for (size_t i = root(); i < _elems.size(); i++) {
        heaped.push_back(_elems[i]);
    }
}

int heap::size() const {
    return _elems.size();
}

void heap::updateDistance(int csvIdx, double newDistance) {
    // check if csvIdx is a part of the heap
    if (positions_.find(csvIdx) == positions_.end() || positions_[csvIdx] >= (int) _elems.size()) {
        return;
    }

    int heapIdx = positions_[csvIdx];
    _elems[heapIdx].second = newDistance;

    // heapify up or heapify down after updating distance
    size_t parentIdx = parent(heapIdx);
    if (_elems[parentIdx].second < _elems[heapIdx].second) {
        heapifyDown(heapIdx);
    } else {
        heapifyUp(heapIdx);
    }
}

bool heap::isInHeap(int csvIdx) {
    if (positions_.find(csvIdx) == positions_.end() || positions_[csvIdx] == BigNum) {
        return false;
    }
    return true;
}