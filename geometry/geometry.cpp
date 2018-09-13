#include <cmath>
#include <algorithm>
#include <vector>

using namespace std;

#define EPS 1e-12
#define LEFT_TOP POS(1000, 1000)
#define NO_INTERSECT POS(-1234, -1234)
#define PARALLEL POS(-1001, -1001)
#define COLINE POS(1234, 1234)
const double PI = acos(-1.0);

typedef double T;

class POS {
public:
    T x, y;
    POS(const T& x = 0, const T& y = 0) : x(x), y(y) {}
    POS(const POS& x) : x(x.x), y(x.y) {}

    bool operator==(const POS& rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    POS& operator+=(const POS& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    POS operator -() {
        POS tmp(-x, -y);
        return tmp;
    }

    POS const operator+(const POS& rhs) const {
        return POS(*this) += rhs;
    }


    POS const operator-(const POS& rhs) const {
        POS tmp = rhs;
        tmp = -tmp;
        return POS(*this) += (tmp);
    }

    POS operator * (T c) const { return POS(x*c, y*c); }

    POS operator / (T c) const { return POS(x/c, y/c); }

    double dist(const POS& rhs) const {
        T tmp_x = x-rhs.x, tmp_y = y-rhs.y;
        return sqrt(tmp_x*tmp_x+tmp_y*tmp_y);
    }

    friend ostream& operator<<(ostream& out, const POS& pos) {
        out << pos.x << " " << pos.y;
        return out;
    }
};


T dot(POS p, POS q)     { return p.x*q.x+p.y*q.y; }

T dist2(POS p, POS q)   { return dot(p-q,p-q); }

// rotate a point CCW or CW around the origin
POS RotateCCW90(POS p)   { return POS(-p.y,p.x); }

POS RotateCW90(POS p)    { return POS(p.y,-p.x); }

POS RotateCCW(POS p, double t) {
  return POS(p.x*cos(t)-p.y*sin(t), p.x*sin(t)+p.y*cos(t));
}

// project point c onto line through a and b
// assuming a != b
POS ProjectPointLine(POS a, POS b, POS c) {
  return a + (b-a)*dot(c-a, b-a)/dot(b-a, b-a);
}

// project point c onto line segment through a and b
POS ProjectPointSegment(POS a, POS b, POS c) {
  double r = dot(b-a,b-a);
  if (fabs(r) < EPS) return a;
  r = dot(c-a, b-a)/r;
  if (r < 0) return a;
  if (r > 1) return b;
  return a + (b-a)*r;
}

// compute distance between point (x,y,z) and plane ax+by+cz=d
T DistancePointPlane(T x, T y, T z, T a, T b, T c, T d) {
  return fabs(a*x+b*y+c*z-d)/sqrt(a*a+b*b+c*c);
}

bool cmp_convex(const POS& lhs, const POS& rhs) {
    return (lhs.x < rhs.x) || ( (lhs.x == rhs.x)&&(lhs.y < rhs.y) );
}

inline T cross(const POS& o, const POS& a, const POS& b) {
    double value = (a.x-o.x)*(b.y-o.y) - (a.y-o.y)*(b.x-o.x);
    if (fabs(value) < EPS) return 0;
    return value;
}

void convex_hull(POS* points, POS* need, int& n) {
    sort(points, points+n, cmp_convex);
    int index = 0;
    for (int i = 0; i < n; ++i) {
        while (index >= 2 && cross(need[index-2], need[index-1], points[i]) <= 0) index--;
        need[index++] = points[i];
    }
    int half_point = index+1;
    for (int i = n-2; i >= 0; --i) {
        while (index >= half_point && cross(need[index-2], need[index-1], points[i]) <= 0) index--;
        need[index++] = points[i];
    } /* be careful that start point will appear in fisrt and last in need array */
    n = index;
}

class LINE {
public:
    POS start, end, vec;
    double angle;
    LINE() {}
    LINE(const T& st_x, const T& st_y, const T& ed_x, const T& ed_y) :
        start(st_x, st_y), end(ed_x, ed_y), vec(end - start), angle(atan2(vec.x, vec.y)) {}

    LINE(const POS& start, const POS& end) :
        start(start), end(end), vec(end - start), angle(atan2(vec.x, vec.y)) {}

    LINE(const POS& end) : /* start point is origin */
        start(0, 0), end(end), vec(end), angle(atan2(vec.x, vec.y)) {}

    LINE(const T a, const T b, const T c) : /* given line by ax+by+c = 0 */
        start(0, 0), end(0, 0), vec(-b, a) {
        if (a == 0) {
            start.y = end.y = -c/b;
            end.x = -b;
        }
        else if (b == 0) {
            start.x = end.x = -c/a;
            end.y = a;
        }
        else if (c == 0) {
            end.x = -b; end.y = a;
        }
        else {
            start.y = -c/b; end.x = -c/a;
            vec.x = -c/a; vec.y = c/b;
        }
        angle = atan2(vec.x, vec.y);
    }

    LINE build_orthogonal(const POS& point) const {
        T c = -(vec.x*point.x + vec.y*point.y);
        return LINE(vec.x, vec.y, c);
    }

    T length2() const { /* square */
        T x = start.x - end.x, y = start.y - end.y;
        return x*x + y*y;
    }

    void modify(T x, T y) {
        this->end.x += x;
        this->end.y += y;
        this->vec.x += x;
        this->vec.y += y;
    }

    bool on_line(const POS& a) const {
        if (vec.x == 0) {
            if (start.x != a.x) return false;
            return true;
        }
        if (vec.y == 0) {
            if (start.y != a.y) return false;
            return true;
        }
        return fabs(( (a.x-start.x)/vec.x*vec.y + start.y )- a.y) < EPS;
    }

    bool operator/(const LINE& rhs) const { /* to see if this line parallel to LINE rhs */
        return (vec.x*rhs.vec.y == vec.y*rhs.vec.x);
    }

    bool operator==(const LINE& rhs) const { /* to see if they are same line */
        return (*this/rhs) && (rhs.on_line(start));
    }

    POS intersect(const LINE& rhs) const {
        if (*this==rhs) return COLINE; /* return co-line */
        if (*this/rhs) return PARALLEL; /* return parallel */

        double A1 = vec.y, B1 = -vec.x, C1 = end.x*start.y - start.x*end.y;
        double A2 = rhs.vec.y, B2 = -rhs.vec.x, C2 = rhs.end.x*rhs.start.y - rhs.start.x*rhs.end.y;
        return POS( (B2*C1-B1*C2)/(A2*B1-A1*B2), (A1*C2-A2*C1)/(A2*B1-A1*B2) ); /* sometimes has -0 */
    }

    double dist(const POS& a) const {
        return fabs(vec.y*a.x - vec.x*a.y + vec.x*start.y - vec.y*start.x)/sqrt(vec.y*vec.y+vec.x*vec.x);
    }

    double dist(const LINE& rhs) const {
        POS intersect_point = intersect(rhs);
        if (intersect_point == PARALLEL) {
            return dist(rhs.start);
        }
        return 0;
    }

    friend ostream& operator<<(ostream& out, const LINE& line) {
        out << line.start << "-->" << line.end << " vec: " << line.vec;
        return out;
    }
};

POS ComputeCircleCenter(POS a, POS b, POS c) {
  b=(a+b)/2;
  c=(a+c)/2;
  LINE l1 = LINE(b, b+RotateCW90(a-b));
  LINE l2 = LINE(c, c+RotateCW90(a-c));
  return l1.intersect(l2);
}

class LINESEG : public LINE {
public:
    LINESEG() : LINE(POS(0, 0)) {}
    LINESEG(const LINE& input) : LINE(input) {}
    LINESEG(const POS& start, const POS& end) : LINE(start, end) {}

    bool on_lineseg(const POS& a) const {
        if (!on_line(a)) return false;
        bool first, second;
        if (vec.x >= 0) first = (a.x >= start.x)&&(a.x <= end.x);
        else first = (a.x <= start.x)&&(a.x >= end.x);
        if (vec.y >= 0) second = (a.y >= start.y)&&(a.y <= end.y);
        else second = (a.y <= start.y)&&(a.y >= end.y);
        return first&&second;
    }

    bool operator==(const LINESEG& rhs) const {
        return ( (rhs.start == start && rhs.end == end) ||
              (rhs.start == end && rhs.end == start) );
    }

    bool operator==(const LINE& rhs) const {
        return this->LINE::operator==(rhs);
    }

    T dot(const LINESEG& rhs) const {
        return vec.x*rhs.vec.x + vec.y*rhs.vec.y;
    }

    T cross(const LINESEG& rhs) const {
        return vec.x*rhs.vec.y - vec.y*rhs.vec.x;
    }

    bool clockwise(const LINE& a) const { /* to see if LINE a is in b's clockwise way */
        return cross(a) > 0;
    }

    double dist(const POS& a) const {
        double ortho_dist = this->LINE::dist(a);
        LINE ortho_line = build_orthogonal(a);
        POS intersect_point = this->LINE::intersect(ortho_line);
        if (on_lineseg(intersect_point)) return ortho_dist;
        else return min(a.dist(this->start), a.dist(this->end));
    }

    double dist(const LINE& line) const {
        POS intersect_point = this->LINE::intersect(line);
        if (intersect_point == COLINE) return 0;
        if (intersect_point == PARALLEL) return dist(line.start);
        if (on_lineseg(intersect_point)) return 0;
        return min(line.dist(start), line.dist(end));
    }

    double dist(const LINESEG& line) const {
        return min( min(dist(line.start), dist(line.end)),
                    min(line.dist(start), line.dist(end)) );
    }

    POS intersect(const LINESEG& rhs) const {
        LINE a1b1(start, rhs.start);
        LINE a1b2(start, rhs.end);
        LINE b1a1(rhs.start, start);
        LINE b1a2(rhs.start, end);

        POS tmp(this->LINE::intersect(rhs));

        if (tmp == COLINE) {
            if ( (start==rhs.start) && (!rhs.on_lineseg(end)) && (!on_lineseg(rhs.end)) ) return start;
            if ( (start==rhs.end) && (!rhs.on_lineseg(end)) && (!on_lineseg(rhs.start)) ) return start;
            if ( (end==rhs.start) && (!rhs.on_lineseg(start)) && (!on_lineseg(rhs.end)) ) return end;
            if ( (end==rhs.end) && (!rhs.on_lineseg(start)) && (!on_lineseg(rhs.start)) ) return end;
            if (on_lineseg(rhs.start) || on_lineseg(rhs.end) || rhs.on_lineseg(start) || rhs.on_lineseg(end)) return COLINE;
            return NO_INTERSECT;
        }

        bool intersected =  ( (cross(a1b1)*cross(a1b2)<=0) && (rhs.cross(b1a1)*rhs.cross(b1a2)<=0) );
        if (!intersected) return NO_INTERSECT;
        if (!on_lineseg(tmp) || !rhs.on_lineseg(tmp)) return NO_INTERSECT;
        return tmp;
    }
};

inline bool cmp_half_plane(const LINE &a,const LINE &b){
    if(fabs(a.angle-b.angle) < EPS) return cross(a.start, a.end, b.start) < 0;
    return a.angle > b.angle;
}

void half_plane_intersection(LINE* a, LINE* need, POS* answer, int &n){
    int m = 1, front = 0, rear = 1;
    sort(a, a+n, cmp_half_plane);
    for(int i = 1; i < n; ++i){
        if( fabs(a[i].angle-a[m-1].angle) > EPS ) a[m++] = a[i];
    }
    need[0] = a[0], need[1] = a[1];
    for(int i = 2; i < m; ++i){
        while (front<rear&&cross(a[i].start, a[i].end, need[rear].intersect(need[rear-1]))<0) rear--;
        while (front<rear&&cross(a[i].start, a[i].end, need[front].intersect(need[front+1]))<0) front++;
        need[++rear] = a[i];
    }
    while (front<rear&&cross(need[front].start,need[front].end, need[rear].intersect(need[rear-1]))<0) rear--;
    while (front<rear&&cross(need[rear].start,need[rear].end, need[front].intersect(need[front+1]))<0) front++;
    if (front==rear) return;

    n = 0;
    for (int i=front; i<rear; ++i) answer[n++] = need[i].intersect(need[i+1]);
    if(rear>front+1) answer[n++] = need[front].intersect(need[rear]);
}

void rotating_calipers(int& ans, POS* need, int& n) {
    --n;
    if (n == 2) {
        ans = need[0].dist(need[1]);
        return;
    }

    int now = 2;
    for (int i = 0; i < n; ++i) {
        LINE target(need[i], need[i+1]);
        double pre = target.dist(need[now]);
        for (; now != i; now = (now+1)%(n)) {
            double tmp = target.dist(need[now]);
            if (tmp < pre) break;
            pre = tmp;
        }
        now = (now-1+n)%n;
        ans = max(ans, pre);
    }
}

// determine if point is in a possibly non-convex polygon (by William
// Randolph Franklin); returns 1 for strictly interior points, 0 for
// strictly exterior points, and 0 or 1 for the remaining points.
// Note that it is possible to convert this into an *exact* test using
// integer arithmetic by taking care of the division appropriately
// (making sure to deal with signs properly) and then by writing exact
// tests for checking point on polygon boundary
bool PointInPolygon(const vector<POS> &p, POS q) {
  bool c = 0;
  for (int i = 0; i < p.size(); i++){
    int j = (i+1)%p.size();
    if ((p[i].y <= q.y && q.y < p[j].y ||
      p[j].y <= q.y && q.y < p[i].y) &&
      q.x < p[i].x + (p[j].x - p[i].x) * (q.y - p[i].y) / (p[j].y - p[i].y))
      c = !c;
  }
  return c;
}

// determine if point is on the boundary of a polygon
bool PointOnPolygon(const vector<POS> &p, POS q) {
  for (int i = 0; i < p.size(); i++)
    if (dist2(ProjectPointSegment(p[i], p[(i+1)%p.size()], q), q) < EPS)
      return true;
    return false;
}

// compute intersection of line through points a and b with
// circle centered at c with radius r > 0
vector<POS> CircleLineIntersection(POS a, POS b, POS c, double r) {
  vector<POS> ret;
  b = b-a;
  a = a-c;
  double A = dot(b, b);
  double B = dot(a, b);
  double C = dot(a, a) - r*r;
  double D = B*B - A*C;
  if (D < -EPS) return ret;
  ret.push_back(c+a+b*(-B+sqrt(D+EPS))/A);
  if (D > EPS)
    ret.push_back(c+a+b*(-B-sqrt(D))/A);
  return ret;
}

// compute intersection of circle centered at a with radius r
// with circle centered at b with radius R
vector<POS> CircleCircleIntersection(POS a, POS b, double r, double R) {
  vector<POS> ret;
  double d = sqrt(dist2(a, b));
  if (d > r+R || d+min(r, R) < max(r, R)) return ret;
  double x = (d*d-R*R+r*r)/(2*d);
  double y = sqrt(r*r-x*x);
  POS v = (b-a)/d;
  ret.push_back(a+v*x + RotateCCW90(v)*y);
  if (y > 0)
    ret.push_back(a+v*x - RotateCCW90(v)*y);
  return ret;
}

// This code computes the area or centroid of a (possibly nonconvex)
// polygon, assuming that the coordinates are listed in a clockwise or
// counterclockwise fashion.  Note that the centroid is often known as
// the "center of gravity" or "center of mass".
double ComputeSignedArea(const vector<POS> &p) {
  double area = 0;
  for(int i = 0; i < p.size(); i++) {
    int j = (i+1) % p.size();
    area += p[i].x*p[j].y - p[j].x*p[i].y;
  }
  return area / 2.0;
}

double ComputeArea(const vector<POS> &p) {
  return fabs(ComputeSignedArea(p));
}

POS ComputeCentroid(const vector<POS> &p) {
  POS c(0,0);
  double scale = 6.0 * ComputeSignedArea(p);
  for (int i = 0; i < p.size(); i++){
    int j = (i+1) % p.size();
    c = c + (p[i]+p[j])*(p[i].x*p[j].y - p[j].x*p[i].y);
  }
  return c / scale;
}

// tests whether or not a given polygon (in CW or CCW order) is simple
bool IsSimple(const vector<POS> &p) {
  for (int i = 0; i < p.size(); i++) {
    for (int k = i+1; k < p.size(); k++) {
      int j = (i+1) % p.size();
      int l = (k+1) % p.size();
      if (i == l || j == k) continue;
      LINESEG l1 = LINESEG(p[i], p[j]), l2 = LINESEG(p[k], p[l]);
      POS res = l1.intersect(l2);
      if (!(res == NO_INTERSECT))
        return false;
      //if (SegmentsIntersect(p[i], p[j], p[k], p[l]))
      //  return false;
    }
  }
  return true;
}
