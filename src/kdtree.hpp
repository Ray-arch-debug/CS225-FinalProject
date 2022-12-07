/**
 * @file kdtree.cpp
 * Implementation of KDTree class.
 */

#include <utility>
#include <algorithm>
#include <stack>

using namespace std;

template <int Dim>
bool KDTree<Dim>::smallerDimVal(const Point<Dim>& first,
                                const Point<Dim>& second, int curDim) const
{
    /**
     * @todo Implement this function!
     */
  if (first[curDim] == second[curDim]) {
    return first < second;
  }
  return first[curDim] < second[curDim];

}

template <int Dim>
bool KDTree<Dim>::shouldReplace(const Point<Dim>& target,
                                const Point<Dim>& currentBest,
                                const Point<Dim>& potential) const
{
    /**
     * @todo Implement this function!
     */
     int potential_dist = squaredDistance(potential, target);
     int current_dist = squaredDistance(currentBest, target);

     return (potential_dist < current_dist) || (potential_dist == current_dist && potential < currentBest);
}

template <int Dim>
KDTree<Dim>::KDTree(const vector<Point<Dim>>& newPoints)
{
    /**
     * @todo Implement this function!
     */
     vector<Point<Dim>> points = newPoints;
     buildTree(points, root, 0, 0, newPoints.size());
}

template <int Dim>
KDTree<Dim>::KDTree(const KDTree<Dim>& other) {
  /**
   * @todo Implement this function!
   */
   auto newPoints = other.getPointsVector();
   buildTree( newPoints, root, 0, 0, newPoints.size() );
}

template <int Dim>
const KDTree<Dim>& KDTree<Dim>::operator=(const KDTree<Dim>& rhs) {
  /**
   * @todo Implement this function!
   */
  if (this == &rhs) { return *this; }
  clear();
  auto newPoints = rhs.getPointsVector();
  buildTree( newPoints, root, 0, 0, newPoints.size() );
  return *this;
}

template <int Dim>
KDTree<Dim>::~KDTree() {
  /**
   * @todo Implement this function!
   */
   clear(root);
}

template <int Dim>
Point<Dim> KDTree<Dim>::findNearestNeighbor(const Point<Dim>& query) const
{
    /**
     * @todo Implement this function!
     */
  unsigned dim_index = 0;
  return findNearestNeighborSearch(query, root, dim_index);
}

template <int Dim>
int KDTree<Dim>::squaredDistance(Point<Dim> first, Point<Dim> second) {
  int sum = 0;
  for (int i = 0; i < Dim; i++) {
    sum += (first[i] - second[i]) * (first[i] - second[i]);
  }
  return sum;
}

template <int Dim>
void KDTree<Dim>::buildTree(vector<Point<Dim>>& newPoints, KDTreeNode*& node, unsigned dim_index, unsigned start, unsigned end) {
  if (start >= end) { return; }
  unsigned mid = (start + end - 1) / 2;
  quickSelect(newPoints, dim_index, mid, start, end);
  node = new KDTreeNode(newPoints.at(mid));
  dim_index = (dim_index+1) % Dim;
  buildTree(newPoints, node->left, dim_index, start, mid);
  buildTree(newPoints, node->right, dim_index, mid+1, end);
}

template <int Dim>
unsigned KDTree<Dim>::quickSelect(vector<Point<Dim>>& newPoints, unsigned dim_index, unsigned mid, unsigned start, unsigned end) {
  if (start == end-1) { return start; }
  std::swap(newPoints.at(mid), newPoints.at(end-1));
  unsigned left = start;
  unsigned right = end-1;
  const Point<Dim>& pivot_point = newPoints.at(end-1);
  while (left < right) {
    const Point<Dim>& left_point = newPoints.at(left);
    const Point<Dim>& right_point = newPoints.at(right);
    if (left_point[dim_index] < pivot_point[dim_index] || (left_point[dim_index] == pivot_point[dim_index] && left_point < pivot_point) ) {
      left++;
      continue;
    }
    if (pivot_point[dim_index] < right_point[dim_index] || (pivot_point[dim_index] == right_point[dim_index] && pivot_point < right_point) ) {
      right--;
      continue;
    }
    std::swap(newPoints.at(left), newPoints.at(right));
  }

  // left == right
  const Point<Dim>& left_point = newPoints.at(left);
  if (left_point[dim_index] < pivot_point[dim_index] || (left_point[dim_index] == pivot_point[dim_index] && left_point < pivot_point) ) {
    std::swap(newPoints.at(left), newPoints.at(end-1));
  }

  if (left == mid) { return mid; }
  if (left < mid) {
    return quickSelect(newPoints, dim_index, mid-(left-start), left, end) + left - start;
  } else {
    return quickSelect(newPoints, dim_index, mid, start, left);
  }
}

template <int Dim>
void KDTree<Dim>::clear(KDTreeNode*& node) {
  if (node == NULL) { return; }
  clear(node->left);
  clear(node->right);
  delete node;
  node = NULL;
}

template <int Dim>
const vector<Point<Dim>>& KDTree<Dim>::getPointsVector() const {
  vector<Point<Dim>> vector;
  std::stack<KDTreeNode*> stack;
  if (root == NULL) { return vector; }
  stack.push(root);
  while (!stack.empty()) {
    KDTreeNode *node = stack.top();
    stack.pop();
    vector.push_back(node->point);
    if (node->left != NULL) { stack.push_back(node->left); }
    if (node->right != NULL) { stack.push_back(node->right); }
  }
  return vector;
}

template <int Dim>
Point<Dim> KDTree<Dim>::findNearestNeighborSearch(const Point<Dim>& query, KDTreeNode* node, unsigned& dim_index) const {
  std::stack<KDTreeNode*> stack;
    for (KDTreeNode* curr = node; curr != NULL; incDim(dim_index)) {
      stack.push(curr);
      if (curr->point == query) { return curr->point; }
      if ( query[dim_index] < curr->point[dim_index] || (query[dim_index] == curr->point[dim_index] && query < curr->point) ) {
        curr = curr->left;
      } else {
        curr = curr->right;
      }
    }

    Point<Dim> closest = stack.top()->point;
    while (!stack.empty()) {
      KDTreeNode* curr = stack.top();
      stack.pop();
      decDim(dim_index);
      if ( (query[dim_index] - curr->point[dim_index]) * (query[dim_index] - curr->point[dim_index]) <= squaredDistance(query, closest)
      // should be if the hypersphere intersects the other side...
      ) {
        if (shouldReplace(query, closest, curr->point)) { closest = curr->point; }
        if ( query[dim_index] < curr->point[dim_index] || (query[dim_index] == curr->point[dim_index] && query < curr->point) ) {
          // search curr's right subtree, left subtree already searched
          if (curr->right != NULL) {
            incDim(dim_index);
            Point<Dim> closest_in_subtree = findNearestNeighborSearch(query, curr->right, dim_index);
            decDim(dim_index);
            if (shouldReplace(query, closest, closest_in_subtree)) { closest = closest_in_subtree; }
          }
        } else {
          // search curr's left subtree, right subtree already searched
          if (curr->left != NULL) {
            incDim(dim_index);
            Point<Dim> closest_in_subtree = findNearestNeighborSearch(query, curr->left, dim_index);
            decDim(dim_index);
            if (shouldReplace(query, closest, closest_in_subtree)) { closest = closest_in_subtree; }
          }
        }
      }
    }
    return closest;
}

template <int Dim>
const std::vector<Point<Dim>>& KDTree<Dim>::findWithinDistance(const Point<Dim>& query, double distance) const {
  // TODO
  return std::vector<Point<Dim>>();
}

template <int Dim>
void KDTree<Dim>::incDim(unsigned& dim_index) const {
  if (dim_index < Dim - 1) {
    dim_index++;
  } else {
    dim_index = 0;
  }
}

template <int Dim>
void KDTree<Dim>::decDim(unsigned& dim_index) const {
  if (dim_index > 0) {
    dim_index--;
  } else {
    dim_index = Dim - 1;
  }
}