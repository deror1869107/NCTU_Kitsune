#include<algorithm>
#include<cmath>
#include<cstdio>
#include<queue>
#include<cstdlib>
#include<vector>
#define MAXN 50100
using namespace std;
inline long long sq(long long x){return x*x;}
const double alpha=0.75;
int W,H,rx[MAXN],ry[MAXN];
namespace KDTree{
  struct Point {
    int x,y;
    int index;
    long long distance(const Point &b)const{
      return sq(x-b.x) + sq(y-b.y);
    }
    bool operator==(const Point& rhs){return index==rhs.index;}
  };
  struct qnode{
    Point p;
    long long dis;
    qnode(){}
    qnode(Point _p,long long _dis){
      p = _p;
      dis = _dis;
    }
    bool operator <(const qnode &b)const{
      if(dis != b.dis)return dis < b.dis;
      else return p.index < b.p.index;
    }
  };
  priority_queue<qnode>q;
  inline bool cmpX(const Point &a,const Point &b){
    return a.x < b.x || (a.x == b.x && a.y < b.y) || (a.x == b.x && a.y == b.y && a.index < b.index);
  }
  inline bool cmpY(const Point &a,const Point &b){
    return a.y < b.y || (a.y == b.y && a.x < b.x) || (a.y == b.y && a.x == b.x && a.index < b.index);
  }
  bool cmp(const Point &a,const Point &b,bool div){
    return div?cmpY(a,b):cmpX(a,b);
  }
  struct Node{
    Point e;
    Node *lc,*rc;
    int size;
    bool div;
    inline void pull(){
      size = 1 + lc->size + rc->size;
    }
    inline bool isBad(){
      return lc->size > alpha*size || rc->size > alpha*size;
    }
  }pool[MAXN],*tail,*root,*recycle[MAXN],*null;
  int rc_cnt;
  void init(){
    tail = pool;
    null = tail++;
    null->lc = null->rc = null;
    null->size = 0;
    rc_cnt = 0;
    root = null;
  }
  Node *newNode(Point e){
    Node *p;
    if(rc_cnt)p = recycle[--rc_cnt];
    else p = tail++;
    p->e = e;
    p->lc = p->rc = null;
    p->size = 1;
    return p;
  }
  Node *build(Point *a,int l,int r,bool div){
    if(l >= r)return null;
    int mid = (l+r)/2;
    nth_element(a+l,a+mid,a+r,div?cmpY:cmpX);
    Node *p = newNode(a[mid]);
    p->div = div;
    p->lc = build(a,l,mid,!div);
    p->rc = build(a,mid+1,r,!div);
    p->pull();
    return p;
  }
  void getTree(Node *p,vector<Point>& v){
    if(p==null) return;
    getTree(p->lc,v);
    v.push_back(p->e);
    recycle[rc_cnt++]=p;
    getTree(p->rc,v);
  }
  Node *rebuild(vector<Point>& v,int l,int r,bool div){
    if(l>=r) return null;
    int mid = (l+r)/2;
    nth_element(v.begin()+l,v.begin()+mid,v.begin()+r,div?cmpY:cmpX);
    Node *p = newNode(v[mid]);
    p->div = div;
    p->lc = rebuild(v,l,mid,!div);
    p->rc = rebuild(v,mid+1,r,!div);
    p->pull();
    return p;
  }
  void rebuild(Node *&p){
    vector<Point> v;
    getTree(p,v);
    p = rebuild(v,0,v.size(),p->div);
  }
  Node **insert(Node *&p,Point a,bool div){
    if(p==null){
      p = newNode(a);
      p->div = div;
      return &null;
    }
    else{
      Node **res;
      if(cmp(a,p->e,div)) res=insert(p->lc,a,!div);
      else res=insert(p->rc,a,!div);
      p->pull();
      if(p->isBad()) res=&p;
      return res;
    }
  }
  void insert(Point e){
    Node **p = insert(root,e,0);
    if(*p!=null) rebuild(*p);
  }
  Node **get_min(Node *&p,bool div){
    if(p->div==div){
      if(p->lc!=null) return get_min(p->lc,div);
      else return &p;
    }
    else{
      Node **res=&p,**tmp;
      if(p->lc!=null){
        tmp = get_min(p->lc,div);
        if(cmp((*tmp)->e,(*res)->e,div)) res=tmp;
      }
      if(p->rc!=null){
        tmp = get_min(p->rc,div);
        if(cmp((*tmp)->e,(*res)->e,div)) res=tmp;
      }
      return res;
    }
  }
  void del(Node *&p){
    Node **nxt;
    if(p->rc!=null){
      nxt = get_min(p->rc,p->div);
      p->e = (*nxt)->e;
      del(*nxt);
    }
    else if(p->lc!=null){
      nxt = get_min(p->lc,p->div);
      p->e = (*nxt)->e;
      del(*nxt);
      p->rc = p->lc;
      p->lc = null;
    }
    else{
      recycle[rc_cnt++]=p;
      p=null;
    }
  }
  void del(Node *&p,Point d){
    if(p->e==d){
      del(p);
    }
    else if(cmp(d,p->e,p->div)) del(p->lc,d);
    else del(p->rc,d);
  }
  void search(Point p,Node *t,bool div,int m){
    if(!t)return;
    if(cmp(p,t->e,div)){
      search(p,t->lc,!div,m);
      if(q.size() < m){
        q.push(qnode(t->e,p.distance(t->e)));
        search(p,t->rc,!div,m);
      }
      else {
        if(p.distance(t->e) <= q.top().dis){
          q.push(qnode(t->e,p.distance(t->e)));
          q.pop();
        }
        if(!div){
          if(sq(t->e.x-p.x) <= q.top().dis)
            search(p,t->rc,!div,m);
        }
        else {
          if(sq(t->e.y-p.y) <= q.top().dis)
            search(p,t->rc,!div,m);
        }
      }
    }
    else {
      search(p,t->rc,!div,m);
      if(q.size() < m){
        q.push(qnode(t->e,p.distance(t->e)));
        search(p,t->lc,!div,m);
      }
      else {
        if(p.distance(t->e) <= q.top().dis){
          q.push(qnode(t->e,p.distance(t->e)));
          q.pop();
        }
        if(!div){
          if(sq(t->e.x-p.x) <= q.top().dis)
            search(p,t->lc,!div,m);
        }
        else {
          if(sq(t->e.y-p.y) <= q.top().dis)
            search(p,t->lc,!div,m);
        }
      }
    }
  }
  void search(Point p,int m){
    while(!q.empty())q.pop();
    search(p,root,0,m);
  }
  void getRange(Node *p,vector<Point>& v,int x1,int x2,int y1,int y2){
    if(p==null) return;
    if(x1<=p->e.x && p->e.x<=x2 && y1<=p->e.y && p->e.y<=y2) v.push_back(p->e);
    if(p->div ? y1<=p->e.y : x1<=p->e.x) getRange(p->lc,v,x1,x2,y1,y2);
    if(p->div ? y2>=p->e.y : x2>=p->e.x) getRange(p->rc,v,x1,x2,y1,y2);
  }
  void solve(Point p){
    del(root,p);
    insert(p);
  }
};
KDTree::Point p[MAXN];
int main(){
  KDTree::init();
  KDTree::root = KDTree::build(p,0,n,0);
  while(q--){
    KDTree::Point tmp,p1,p2;
    scanf("%d%d",&tmp.x,&tmp.y);
    search(tmp,2);
    p1=KDTree::q.top().p;
    KDTree::q.pop();
    p2=KDTree::q.top().p;
    KDTree::q.pop();
  }
  return 0;
}
