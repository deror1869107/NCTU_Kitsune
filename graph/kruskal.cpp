// O(E * logV)
#include "disjoint_set.hpp"

struct E {int len, node1, node2};
vector<E> v, mst;

bool cmp(const E& l, const E& r) {
  return l.len < r.len;
}

void kruskal()
{
  init();
  sort(v.begin, v.end, cmp);  // sort edges by length
    for(auto e: v)
      if(union(e.node1, e.node2))
          mst.push_back(e); // put selected edge in MST
}
